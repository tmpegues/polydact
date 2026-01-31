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

        self.goal_pub = self.create_publisher(Goal, 'serial_goal', 10)

        self.s_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.timer = self.create_timer(1 / 1000, self.timer_callback)

    def timer_callback(self):
        """Run the serial reader and publish to topic `serial_goal`."""
        # if self.s_port.in_waiting > 0:
        line2 = self.s_port.readline().decode('utf-8').strip()
        if line2:
            self.goal_pub.publish(Goal(id=2, goal=int(line2)))
            self.get_logger().info(f'message: {line2}')


def main(args=None):
    """Run the serial_reader node."""
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
