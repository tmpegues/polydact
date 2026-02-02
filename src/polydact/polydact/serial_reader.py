"""Read the serial monitor from the Pico 2 to publish a motor goal."""

from polydact_interfaces.msg import Goal
import rclpy
from rclpy.node import Node

import serial


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

        self.goal_pub = self.create_publisher(Goal, 'motor_goal', 10)

        self.s_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.timer = self.create_timer(1 / 1000, self.timer_callback)

    def timer_callback(self):
        """Run the serial reader and publish to topic `serial_goal`."""
        line = self.s_port.readline().decode('utf-8').strip()
        self.get_logger().info(f'{line}')
        if line:
            id = int(line[0])
            raw_value = int(line[2:])

            value = raw_value - ((self.high + self.low) / 2)  # Center the readings at 0
            self.get_logger().debug(f'raw: {raw_value}, value: {value}')
            value /= (
                (self.high - self.low) / 2 * (1 / 10)
            )  # Squash the values down to -1 to 1, then expand to -10 to 10
            if value > 10:
                value = 10
            elif value < -10:
                value = -10
            self.get_logger().debug(f'raw: {raw_value}, value: {value}')
            value = int(value)

            self.reads[id].append(value)
            self.reads[id].pop(0)

            self.get_logger().debug(f'message: {line}')
            self.get_logger().debug(f'id: {id}, value: {value}')
            self.get_logger().debug(f'{self.reads[id]}')

            self.goal_pub.publish(Goal(id=id, goal=int(sum(self.reads[id]) / self.smoothing)))


def main(args=None):
    """Run the serial_reader node."""
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
