"""Read the serial monitor from the Pico 2 to publish a motor goal."""

from polydact_interfaces.msg import MotorState
from polydact_interfaces.srv import Mode
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from polydact.sensor import Sensor

import serial


class SerialReader(Node):
    """Reads the serial monitor and publishes motor goals."""

    def __init__(self):
        """Initialize serial reader and goal publisher."""
        super().__init__('serial_reader')

        # Creating publisher first to that PlotJuggler doesn't have to wait for calibration
        self.goal_pub = self.create_publisher(
            MotorState,
            'motor_goal',
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

        # Get motor list and initialize sensors
        self.declare_parameter('sensor_ids', [1, 2, 3])
        sensor_ids = self.get_parameter('sensor_ids').value

        self.declare_parameter('motor_ids', [1, 2, 3])
        self.motor_ids = self.get_parameter('motor_ids').value

        self.sensors = {}
        for sensor_id, motor_id in zip(sensor_ids, self.motor_ids):
            self.sensors.update({sensor_id: Sensor(sensor_id, motor_id)})

        # Set up serial port
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        port = self.get_parameter('serial_port').value
        self.get_logger().info(f'serial port: {port}')

        self.s_port = False
        while not self.s_port:
            try:
                self.s_port = serial.Serial(port, 115200, timeout=10)
            except serial.SerialException:
                self.get_logger().error('Serial device not found.', throttle_duration_sec=1)

        self.set_mode_client = self.create_client(Mode, 'set_mode')
        if not self.set_mode_client.wait_for_service(timeout_set=5.0):
            self.get_logger().error('Failed to find Mode ("set_mode") service')
            raise RuntimeError('Failed to find Mode ("set_mode") service')

        # Calibrate sensor min/max with specified number of readings from each sensor
        self.full_calibration(500)

        # Begin publishing normalized sensor readings
        self.pub_freq = 100
        read_freq = 300
        self.last_published = 0
        # Make sure I'm reading slightly faster than serial is printing
        read_freq *= len(self.sensors) + 0.1

        self.reading_timer = self.create_timer(1 / read_freq, self.reading_timer_callback)
        self.publishing_timer = self.create_timer(
            int(1 / self.pub_freq), self.publishing_timer_callback
        )
        self.get_logger().info('Serial Reader fully initialized')

    def full_calibration(self, num_points: int):
        """
        Calibrate the glove by finding min and max flexes for all fingers, then remap to motors.

        Args:
        ----
        num_points (int): How many points are to be collected for the min max calibration?

        """
        # 1st, all motors should be frozen
        for id, sensor in self.sensors.items():
            sensor.reset()

        self.min_max_calibration(num_points)
        self.map_sensor_to_motor()

    # TODO: Can I pause the publishing timer while calibrating? Doesn't really matter, since I set
    # mode to off at the beginning of this function.
    def map_sensor_to_motor(self):
        """Change the assignment of which sensor controls which motor."""
        # Service to set mode off
        success = False
        while not success:
            success = self.set_mode_client.call_async(Mode.Request(mode=0))

        for motor in self.motor_ids:
            for sensor_id, sensor in self.sensors.keys():
                if sensor.motor_id == motor:
                    self.get_logger().info(
                        f'Motor {motor} is currently assigned to sensor {sensor_id}'
                    )
                    self.get_logger().info(f'Bend the sensor to use with motor {motor} (1 second)')
                    max_bent = 0
                    while True:
                        # Read for 1 second, find max bent sensor
                        max_bent = 0
                        self.get_logger().info(f'Sensor {max_bent} selected for {motor}')
                        self.get_logger().info(f'Keep sensor {max_bent} bent to confirm selection')
                        # Read for another second
                        max_bent2 = 0
                        if max_bent2 == max_bent:
                            break
                        else:
                            self.get_logger().info(
                                f'1st sensor {max_bent} and 2nd sensor {max_bent2} do not match.'
                            )
                else:
                    pass

        success = False
        while not success:
            success = self.set_mode_client.call_async(Mode.Request(mode=1))

    def min_max_calibration(self, num_points: int):
        """
        Read values from the connected sensors for a few seconds to determine ranges.

        Args:
        ----
        num_points (int): How many points will be checked to figure out each sensor's min/max range

        """
        calibrated = 0
        loop = 0
        self.get_logger().info(f'Beginning min/max calibration for {len(self.sensors)} sensors.')
        while calibrated < len(self.sensors) * num_points:
            self.get_logger().info(
                f'Loop {loop} ({calibrated}/{len(self.sensors) * num_points})',
                throttle_duration_sec=1,
            )
            loop += 1
            line = self.get_line()
            if line:
                id, read = line
                if id not in self.sensors:
                    self.get_logger().error(f'Sensor message id {id} is not valid id.')
                    break
                sensor = self.sensors[id]
                if sensor.calibrated >= num_points:
                    # If we've already read many points, do not process any further points
                    pass
                elif sensor.calibrated < num_points:
                    # If we haven't read this many points, check against min and max
                    sensor.calibrated += 1
                    calibrated += 1
                    if read > sensor.max:
                        sensor.max = read
                    elif read < sensor.min:
                        sensor.min = read
                    if sensor.calibrated >= num_points:
                        self.get_logger().info(
                            f'Sensor {sensor.sensor_id} has all {num_points} min/max points read.'
                        )
                        sensor.set_average()
                else:
                    self.get_logger().error(
                        f'Unexpected condition in calibration. Serial line: {line}'
                    )
        self.get_logger().info('Sensor min/max calibration complete:')
        for id, sensor in self.sensors.items():
            self.get_logger().info(f'Sensor {sensor.sensor_id}: {sensor.min} - {sensor.max}')

    def get_line(self):
        """
        Get the most recent serial line.

        Returns
        -------
        (Bool or (int, float)): False if no or invalid serial read, (id, read) if the was valid

        """
        result = False
        line = False
        try:
            line = self.s_port.readline().decode('utf-8').strip()
        except serial.SerialException:
            self.get_logger().error('No device.')
        if line:
            id = int(line[0])
            if id in self.sensors:
                try:
                    read = float(line[2:])
                    result = (id, read)
                    self.get_logger().debug(f'Read: {line}')
                except ValueError:
                    self.get_logger().error(f'Faulty serial line: {line}')

            else:
                self.get_logger().error(f'Unexpected serial line: {line}')
        else:
            self.get_logger().debug('No serial line available.')
        return result

    def reading_timer_callback(self):
        """Read the serial monitor."""
        line = self.get_line()
        if line:
            self.sensors[line[0]].new_read(line[1])

    def publishing_timer_callback(self):
        """Publish all motor goals individually."""
        self.get_logger().debug('Publishing Timer')

        self.goal_pub.publish(
            MotorState(
                id=self.sensors[list(self.sensors.keys())[self.last_published]].motor_id,
                state=self.sensors[list(self.sensors.keys())[self.last_published]].get_value(),
            )
        )
        self.last_published += 1
        self.last_published %= len(self.sensors)


def main(args=None):
    """Run the serial_reader node."""
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
