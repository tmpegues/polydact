"""Simultaneously control three motors to actuate the tentacle digit."""

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Control table address
ADDR_OPERATING_MODE = 11  # Control table address is different in Dynamixel model
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 12


# Protocol version
PROTOCOL_VERSION = 2.0  # Default Protocol version of DYNAMIXEL X series.

# Default settings
DXL_IDs = [2, 3, 5]  # Dynamixel ID : 1
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICE_NAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
CONTROL_MODE  = 4  # 0 = current, 1 = velocity, 3 = position, 4 = extended position, 5 = position + current, 16 = PWM

class MotorCoordinator(Node):
    def __init__(self):
        super().__init__('motor_coordinator')

        self.port_handler = PortHandler(DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open the port!')
            return
        self.get_logger().info('Succeeded to open the port.')

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to set the baudrate!')
            return
        self.get_logger().info('Succeeded to set the baudrate.')

        for id in DXL_IDs:
            self.setup_dynamixel(id)

        self.subscription = self.create_subscription(
            SetPosition,
            'set_position',
            self.set_position_callback, 10)

        self.srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)

    def setup_dynamixel(self, dxl_id):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE, CONTROL_MODE
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set Position Control Mode: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to set Position Control Mode.')

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable torque: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to enable torque.')

    def set_position_callback(self, msg):
        goal_position = msg.position

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, msg.id, ADDR_GOAL_POSITION, goal_position
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'Set [ID: {msg.id}] [Goal Position: {msg.position}]')

    def get_position_callback(self, request, response):
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, request.id, ADDR_PRESENT_POSITION
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'Get [ID: {request.id}] \
                                [Present Position: {dxl_present_position}]')

        response.position = dxl_present_position
        return response

    def __del__(self):
        self.packet_handler.write1ByteTxRx(self.port_handler,
                                        1,
                                        ADDR_TORQUE_ENABLE,
                                        TORQUE_DISABLE)
        self.port_handler.closePort()
        self.get_logger().info('Shutting down motor_coordinator')


def main(args=None):
    rclpy.init(args=args)
    node = MotorCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()