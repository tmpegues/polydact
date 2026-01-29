"""Test different control modes of the Dynamixel."""
from enum import auto, Enum

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
# from dynamixel_sdk_custom_interfaces.msg import SetPosition
# from dynamixel_sdk_custom_interfaces.srv import GetPosition
from polydact_interfaces.msg import Goal
from polydact_interfaces.srv import Mode
import rclpy
from rclpy.node import Node
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
# Changed this to a node parameter DXL_ID = 2  # Dynamixel ID : 1
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICE_NAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
# CONTROL_MODE  = 4  # 1 = velocity, 3 = position, 4 = extended position, 16 = PWM

class Mode(Enum):
    VELOCITY = 1
    POSITION = 3
    EXT_POSITION = 4
    PWM = 16


class MotorCoordinator(Node):
    def __init__(self):
        super().__init__('motor_coordinator')

        self.active = False
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

        self.declare_parameter('ID', 2) # Motor ID
        self.id = self.get_parameter('ID').value

        self.mode = Mode.POSITION
        self.setup_dynamixel(self.id)

        # self.subscription = self.create_subscription(
        #     SetPosition,
        #     'set_position',
        #     self.set_position_callback, 10)

        # self.srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)

        self.goal_sub = self.create_subscription(Goal, 'motor_goal', self.set_goal_cb, 10)
        self.mode_sub = self.create_service(Mode, "set_mode", self.switch_mode_cb)

        self.posi_pub = self.create_publisher(Int16, "position", 10)
        self.velo_pub = self.create_publisher(Int16, "velocity", 10)
        self.load_pub = self.create_publisher(Int16, "load", 10)


        self.timer = self.create_timer(1/10, self.timer_callback)

    def switch_mode_cb(self, request, response):
        """
        Switch the control mode of this node's motor.

        Args:
        ----
        request  (polydact_interfaces/srv/Mode): request.mode contains value to set as the motor's control mode.
        response (polydact_interfaces/srv/Mode): reponse.success is True if the control mode is successfully changed
        """
        response.success = True

        # Filter out invalid values
        if request.mode not in [1, 3, 4, 16]:
            self.get_logger().info(f"{request.mode} is not a valid Control Mode. Use 1 (vel), 3 (pos), 4 (ext pos), or 16 (PWM)")
            response.success = False
        # Turn torque off to that the control mode can be changed
        if response.success:
            response.success = self.toggle_on_off(-1)
        # Change the control mode
        if response.success:
            self.get_logger().info(f"Setting mode: {self.mode.name}")
            response.success = self.set_mode(request.mode)
        # Turn torque back on and change state if mode was successfully changed
        if response.success:
            self.mode = Mode(request.mode)
            response.success = self.toggle_on_off(1)

        return response


    def toggle_on_off(self, new_state = 0) -> bool:
        """
        Toggle the motor on or off

        Args:
        ----
        new_state (int): -1 turns motor off, 1 turns motor on, 0 toggles current state

        Returns
        -------
        (bool): True if the motor state was succesfully toggled_
        """
        success = False
        self.get_logger().debug(f"Motor on/off was {self.active}.")
        if (self.active == True and new_state == 1) or (self.active == False and new_state == -1):
            self.get_logger().debug('Toggle request matches current state')
            success = True
        else:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.id, int(not self.active), TORQUE_DISABLE)

            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Was not able to toggle motor on/off')

            else:
                success = True
                self.active = not self.active
                self.get_logger().debug('Was able to toggle motor on/off')
            self.get_logger().debug(f"Torque on/off is now {self.active}.")
        return success

    def set_mode(self, mode:int) -> bool:
        """
        Set the control mode of the motor.

        Args:
        ----
        mode (int): 1, 3, 4, or 16 to set control mode to velocity, position, extended position, or PWM

        Returns
        -------
        (bool): True if the state was correctly set to the selected mode.
        """
        success = False
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id, ADDR_OPERATING_MODE, mode
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set Control Mode {mode}: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            success = True
            self.get_logger().debug(f'Succeeded to set Control Mode {mode}.')
        return success


    def setup_dynamixel(self, dxl_id):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to disable torque: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to disable torque.')

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE, self.mode
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set Control Mode {self.mode}: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info(f'Succeeded to set Control Mode {self.mode}.')

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

        if self.mode == Mode.POSITION or self.mode == Mode.EXT_POSITION:
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
            self.port_handler, self.id, ADDR_PRESENT_POSITION)

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Position Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Position Error: {self.packet_handler.getRxPacketError(dxl_error)}')

        dxl_present_velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.id, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Velocity Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Velocity Error: {self.packet_handler.getRxPacketError(dxl_error)}')

        dxl_present_load, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.id, ADDR_PRESENT_LOAD)
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