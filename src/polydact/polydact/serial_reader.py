"""Read the serial monitor from the Pico 2 to publish a motor goal."""

from polydact_interfaces.msg import Goal
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
        self.min = 10000
        self.max = 0
        self.smoothing = 10
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
        value = sum(self.reads) / self.smoothing

        value -= (self.max + self.min) / 2  # Center at 0
        value /= (self.max - self.min) / 2  # Squash the values down to -1 to 1

        if value > 10:
            value = 10
        elif value < -10:
            value = -10

        return value


class SerialReader(Node):
    """Reads the serial monitor and publishes motor goals."""

    def __init__(self):
        """Initialize serial reader and goal publisher."""
        super().__init__('serial_reader')
        self.smoothing = 10
        self.reads = {2: [0] * self.smoothing, 3: [0] * self.smoothing, 5: [0] * self.smoothing}
        self.low = 1300
        self.high = 1750
        self.high = (self.low + self.high) / 2

        self.declare_parameter('motor_ids', [2, 3, 5])
        motor_ids = self.get_parameter('motor_ids').value
        self.sensors = {}
        for id in motor_ids:
            self.sensors.update({id: Sensor(id)})

        self.goal_pub = self.create_publisher(Goal, 'motor_goal', 10)

        self.s_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.freq = 1000

        self.extremes = {2: [10000, 0, 0], 3: [1000, 0, 0], 5: [1000, 0, 0]}
        self.calibration_timer = self.create_timer(1 / self.freq, self.calibration)
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)

    def calibration(self):
        """Read values from the connected sensors for a few seconds  to determin ranges."""
        line = self.s_port.readline().decode('utf-8').strip()
        if line:
            pass

    def timer_callback(self):
        """Get the most recent serial line and also publish all motor goals."""
        line = self.s_port.readline().decode('utf-8').strip()
        self.get_logger().info(f'{line}')
        if line:
            id = int(line[0])
            raw_value = float(line[2:])
            self.sensors[id].new_read(raw_value)

        for sensor in self.sensors:
            self.goal_pub.publish(Goal())


def main(args=None):
    """Run the serial_reader node."""
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
