"""Read the serial monitor from the Pico 2 to publish a motor goal."""

from polydact_interfaces.msg import MotorState
from polydact_interfaces.msg import MotorStateArray
import rclpy
from rclpy.node import Node

import serial


class Sensor:
    """Sensor class holds values for each individual sensor."""

    def __init__(self, id):
        """
        Initialize the sensor.

        Args:
        ----
        id (int): Which finger is this sensor on, and which motor does it control?

        """
        self.id = id
        self.min = 10000.0
        self.max = 0.0
        self.smoothing = 15
        self.reads = [0.0] * self.smoothing
        self.calibrated = 0

    def new_read(self, value: float):
        """
        Add the newest raw sensor reading to the rolling list.

        Args:
        ----
        value (float): The new reading to add to the rolling list.

        """
        self.reads.append(value)
        self.reads.pop(0)

    def get_value(self) -> float:
        """
        Get the normalized rolling average of sensor readings.

        Returns
        -------
         (float): The normalized rolling average of sensor readings.

        """
        value = sum(self.reads) / self.smoothing  # Average
        value -= (self.max + self.min) / 2  # Center at 0
        value /= (self.max - self.min) / 2  # Squash the values down to -1 to 1

        # If the max and min are off for some reason, clamp the values
        if value > 1:
            value = 1
        elif value < -1:
            value = -1
        return float(value)

    def set_average(self):
        """Fill the reads list with a neutral value so motors initialize stopped."""
        self.reads = [(self.max + self.min) / 2] * self.smoothing


class SerialReader(Node):
    """Reads the serial monitor and publishes motor goals."""

    def __init__(self):
        """Initialize serial reader and goal publisher."""
        super().__init__('serial_reader')

        # Get motor list and initialize sensors
        self.declare_parameter('motor_ids', [2, 3, 5])
        motor_ids = self.get_parameter('motor_ids').value
        self.sensors = {}
        for id in motor_ids:
            self.sensors.update({id: Sensor(id)})

        # Set up serial port
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        port = self.get_parameter('serial_port').value
        self.s_port = serial.Serial(port, 115200, timeout=1)

        # Calibrate sensor min/max with specified number of readings from each sensor
        self.calibration(100)

        # Begin publishing normalized sensor readings
        self.freq = 50
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)
        self.goal_pub = self.create_publisher(MotorStateArray, 'motor_goal', 10)

    def calibration(self, num_points: int):
        """
        Read values from the connected sensors for a few seconds to determine ranges.

        Args:
        ----
        num_points (int): How many points will be checked to figure out each sensor's min/max range

        """
        calibrated = 0
        loops = 0
        self.get_logger().info(f'Beginning min/max calibration for {len(self.sensors)} sensors.')
        while calibrated < len(self.sensors) * num_points:
            self.get_logger().debug(
                f'Calibrated points: {calibrated} of {len(self.sensors) * num_points}'
            )
            loops += 1
            self.get_logger().debug(f'Loop {loops}')
            line = self.s_port.readline().decode('utf-8').strip()
            if line:
                self.get_logger().debug(f'Serial read: {line}')
                id = int(line[0])
                read = float(line[2:])
                if id not in self.sensors:
                    self.get_logger().error(f'Sensor message id {id} is not valid id.')
                elif self.sensors[id].calibrated >= num_points:
                    # If we've already read many points, do not process any further points
                    pass
                elif self.sensors[id].calibrated < num_points:
                    # If we haven't read this many points, check against min and max
                    self.sensors[id].calibrated += 1
                    calibrated += 1
                    if read > self.sensors[id].max:
                        self.sensors[id].max = read
                    elif read < self.sensors[id].min:
                        self.sensors[id].min = read

                    if self.sensors[id].calibrated >= num_points:
                        self.get_logger().info(
                            f'Sensor {self.sensors[id].id} has all {num_points} min/max points collected.'
                        )
                        self.sensors[id].set_average()
                else:
                    self.get_logger().error(
                        f'Unexpected condition in calibration. Serial line: {line}'
                    )

        self.get_logger().info('Sensor min/max calibration complete:')
        for id, sensor in self.sensors.items():
            self.get_logger().info(f'Sensor {sensor.id}: {sensor.min} - {sensor.max}')

    def timer_callback(self):
        """Get the most recent serial line and also publish all motor goals."""
        line = self.s_port.readline().decode('utf-8').strip()

        if line:
            self.get_logger().debug(f'{line} (timer)')
            id = int(line[0])
            read = float(line[2:])

            if id in self.sensors:
                self.sensors[id].new_read(read)

            else:
                self.get_logger().error('Unexpected serial line: {line}')
        motor_states = MotorStateArray()
        for id, sensor in self.sensors.items():
            motor_states.states.append(MotorState(id=sensor.id, state=sensor.get_value()))
        self.goal_pub.publish(motor_states)
        self.get_logger().info('array published')


def main(args=None):
    """Run the serial_reader node."""
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
