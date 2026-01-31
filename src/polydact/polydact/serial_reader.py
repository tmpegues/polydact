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
        self.reads = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.low = 1300
        self.high = 1750

        self.goal_pub = self.create_publisher(Goal, 'serial_goal', 10)

        self.s_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.timer = self.create_timer(1 / 1000, self.timer_callback)

    def timer_callback(self):
        """Run the serial reader and publish to topic `serial_goal`."""
        line = self.s_port.readline().decode('utf-8').strip()
        self.get_logger().info('')
        if line:
            id = int(line[0])
            raw_value = int(line[2:])

            value = raw_value - ((self.high + self.low) / 2)  # Center the readings at 0
            self.get_logger().info(f'raw: {raw_value}, value: {value}')
            value /= (
                (self.high - self.low) / 2 * (1 / 10)
            )  # Squish the values down to -1 to 1, then expand to -10 to 10
            self.get_logger().info(f'raw: {raw_value}, value: {value}')
            value = int(value)

            self.reads.append(value)
            self.reads.pop(0)

            self.get_logger().info(f'message: {line}')
            self.get_logger().info(f'id: {id}, value: {value}')
            self.get_logger().info(f'{self.reads}')

            self.goal_pub.publish(Goal(id=id, goal=int(sum(self.reads) / 10)))


def main(args=None):
    """Run the serial_reader node."""
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
