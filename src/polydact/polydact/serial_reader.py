"""Read the serial monitor from the Pico 2 to publish a motor goal."""

from polydact_interfaces.msg import MotorState
from polydact_interfaces.msg import MotorStateArray
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

        # Get motor list and initialize sensors
        self.declare_parameter('motor_ids', [1, 2, 3])
        motor_ids = self.get_parameter('motor_ids').value
        self.sensors = {}
        for id in motor_ids:
            self.sensors.update({id: Sensor(id)})

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

        # Calibrate sensor min/max with specified number of readings from each sensor
        self.full_calibration(500)

        self.declare_parameter('array', False)
        self.array = self.get_parameter('array').value
        self.goal_array_pub = self.create_publisher(MotorStateArray, 'motor_goal_array', 10)
        self.goal_pub = self.create_publisher(
            MotorState,
            'motor_goal',
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

        # Begin publishing normalized sensor readings
        self.pub_freq = 100
        read_freq = 300
        if not self.array:
            self.last_published = 0
            read_freq *= 3.1

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

    # TODO: Can I pause the publishing timer while calibrating?
    def map_sensor_to_motor(self):
        """Change the assignment of which sensor controls which motor."""
        for id in self.sensors.keys():
            self.get_logger().info(f'Wiggle the finger you want to use for motor {id}')

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
                            f'Sensor {sensor.sensor_id} has all {num_points} min/max points collected.'
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

        ids = [1, 2, 3]  # self.sensors.keys() ?
        self.goal_pub.publish(
            MotorState(
                id=self.sensors[ids[self.last_published]].motor_id,
                state=self.sensors[ids[self.last_published]].get_value(),
            )
        )
        self.last_published += 1
        self.last_published %= len(self.sensors)
        # for sensor in self.sensors.values():
        #     motor_state = MotorState(id=sensor.id, state=sensor.get_value())
        #     self.goal_pub.publish(motor_state)


def main(args=None):
    """Run the serial_reader node."""
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
