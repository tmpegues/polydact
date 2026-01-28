"""Test different control modes of the Dynamixel."""

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int16

# Control table address
ADDR_OPERATING_MODE = 11  # Control table address is different in Dynamixel model
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128


# Protocol version
PROTOCOL_VERSION = 2.0  # Default Protocol version of DYNAMIXEL X series.

# Default settings
DXL_ID = 2  # Dynamixel ID : 1
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICE_NAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
# CONTROL_MODE  = 4  # 0 = current, 1 = velocity, 3 = position, 4 = extended position, 5 = position + current, 16 = PWM

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
        self.control_mode = 1  # 0 = current, 1 = velocity, 3 = position, 4 = extended position, 5 = position + current, 16 = PWM

        self.setup_dynamixel(DXL_ID)

        self.subscription = self.create_subscription(
            SetPosition,
            'set_position',
            self.set_position_callback, 10)

        self.srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)

        self.posi_pub = self.create_publisher(Int16, "position", 10)
        self.velo_pub = self.create_publisher(Int16, "velocity", 10)
        self.load_pub = self.create_publisher(Int16, "load", 10)

        self.mode_sub = self.create_subscription(Int16, "set_mode", self.mode_switch_cb, 10)
        self.modes = {0:'current', 1:'velocity', 3:'position', 4:'extended position', 5:'position + current', 16: "PWM"}




        self.timer = self.create_timer(1/10, self.timer_callback)

    def mode_switch_cb(self, msg):
        # 0 = current, 1 = velocity, 3 = position, 4 = extended position, 5 = position + current, 16 = PWM
        if msg.data not in [0, 1, 3, 4, 5, 16]:
            self.get_logger().info(f"{msg.data} is not a valid Control Mode")
        else:

            self.control_mode = msg.data
            self.get_logger().info(f"Setting mode: {self.modes[msg.data]}")
            self.setup_dynamixel(DXL_ID)

    def setup_dynamixel(self, dxl_id):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to disable torque: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to disable torque.')

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE, self.control_mode
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set Control Mode {self.control_mode}: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info(f'Succeeded to set Control Mode {self.control_mode}.')

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable torque: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to enable torque.')

    def set_position_callback(self, msg):
        goal = msg.position

        if self.control_mode == 3 or self.control_mode == 4:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler, msg.id, ADDR_GOAL_POSITION, goal
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Error: \
                                        {self.packet_handler.getTxRxResult(dxl_comm_result)}')
            elif dxl_error != 0:
                self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
            else:
                self.get_logger().info(f'Set [ID: {msg.id}] [Goal {self.modes[self.control_mode]}: {msg.position}]')

        elif self.control_mode == 1:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler, msg.id, ADDR_GOAL_VELOCITY, goal
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Error: \
                                        {self.packet_handler.getTxRxResult(dxl_comm_result)}')
            elif dxl_error != 0:
                self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
            else:
                self.get_logger().info(f'Set [ID: {msg.id}] [Goal {self.modes[self.control_mode]}: {msg.position}]')


    def get_position_callback(self, request, response):
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, request.id, ADDR_PRESENT_POSITION
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().debug(f'Get [ID: {request.id}] \
                                [Present Position: {dxl_present_position}]')

        response.position = dxl_present_position
        return response

    def timer_callback(self):
        # Get position, velocity, and current
        dxl_present_position = False
        dxl_present_velocity = False
        dxl_present_load = False

        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, DXL_ID, ADDR_PRESENT_POSITION)

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Position Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Position Error: {self.packet_handler.getRxPacketError(dxl_error)}')

        dxl_present_velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, DXL_ID, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Velocity Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Velocity Error: {self.packet_handler.getRxPacketError(dxl_error)}')

        dxl_present_load, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, DXL_ID, ADDR_PRESENT_LOAD)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Load Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Load Error: {self.packet_handler.getRxPacketError(dxl_error)}')

        self.get_logger().debug(f'\nPos: {dxl_present_position}\nVel: {dxl_present_velocity}\nLoa: {dxl_present_load}')
        self.posi_pub.publish(Int16(data=dxl_present_position))
        self.velo_pub.publish(Int16(data=dxl_present_velocity))
        self.load_pub.publish(Int16(data=dxl_present_load))



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