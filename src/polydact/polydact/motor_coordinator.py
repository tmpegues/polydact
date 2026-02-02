"""Test different control modes of one Dynamixel motor."""

from enum import Enum

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


class MotorMode(Enum):
    """State tracker for the motor's Control Mode."""

    VELOCITY = 1
    POSITION = 3
    EXT_POSITION = 4
    PWM = 16


class SingleMotor(Node):
    """
    Handler for a single Dynamixel motor.

    Publishers:
        + "position" (std_msgs/msg/Int16) - The current position of the motor
        + "velocity" (std_msgs/msg/Int16) - The current velocity of the motor
        + "load" (std_msgs/msg/Int16) - The current load of the motor

    Subscribers:
        + "motor_goal" (polydact_interfaces/msg/Goal) - A goal to set a particular motor to

    Services:
        + "set_mode" (polydact_interfaces/srv/Mode) - What Control Mode to set the motor to.
            ( 1 (vel), 3 (pos), 4 (ext pos), or 16 (PWM)')

    Parameters
    ----------
        + "motor_id" (default 2)- The motor ID to to control with this node
        + "Control_Mode" (default 1, velocity) - The initial Control Mode to use

    """

    def __init__(self):
        """Initialize motor control."""
        super().__init__('single_motor')

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

        self.declare_parameter('motor_id', 2)
        self.id = self.get_parameter('motor_id').value

        self.declare_parameter('Control_Mode', 1)
        self.mode = MotorMode(self.get_parameter('Control_Mode').value)

        self.toggle_on_off(-1)
        self.set_mode(self.mode.value)
        self.toggle_on_off(1)

        self.addresses = {
            'toggle_mode': 11,
            'toggle_torque': 64,
            MotorMode.POSITION: 116,
            MotorMode.VELOCITY: 104,
            'get_pos': 132,
            'get_load': 126,
            'get_vel': 128,
        }

        self.goal_sub = self.create_subscription(Goal, 'motor_goal', self.set_goal_cb, 10)
        self.mode_sub = self.create_service(Mode, 'set_mode', self.switch_mode_cb)

        self.posi_pub = self.create_publisher(Int16, 'position', 10)
        self.velo_pub = self.create_publisher(Int16, 'velocity', 10)
        self.load_pub = self.create_publisher(Int16, 'load', 10)

        self.timer = self.create_timer(1 / 10, self.timer_callback)

    def set_goal_cb(self, msg):
        """
        Set the position, velocity, or PWM goal for the motor.

        Args:
        ----
        msg (polydact_interfaces/msg/Goal): msg.id is the motor to set the goal for
                                            msg.goal is the goal to set to

        """
        self.get_logger().info(f'Goal {msg.goal} received (current mode: {self.mode.name})')
        # if msg.id is not self.id:
        #    return
        self.id = msg.id
        goal = -1 * msg.goal
        success = False
        match self.mode:
            case MotorMode.VELOCITY:
                # Provide a 0 velocity dead zone. Goals come between -10 and 10, set +- 1 to dead?
                if abs(goal) > 3:
                    self.get_logger().info(f'goal pre change{goal}')
                    goal = int(goal / 10 * 300)
                    self.get_logger().info(f'goal post change{goal}')
                    dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                        self.port_handler, msg.id, ADDR_GOAL_VELOCITY, goal
                    )
                    if dxl_comm_result != COMM_SUCCESS:
                        self.get_logger().error(
                            f'Error: \
                                                {self.packet_handler.getTxRxResult(dxl_comm_result)}'
                        )
                    elif dxl_error != 0:
                        self.get_logger().error(
                            f'Error: {self.packet_handler.getRxPacketError(dxl_error)}'
                        )
                    else:
                        success = True
                else:
                    goal = 0
                    dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                        self.port_handler, msg.id, ADDR_GOAL_VELOCITY, goal
                    )
                    if dxl_comm_result != COMM_SUCCESS:
                        self.get_logger().error(
                            f'Error: \
                                                {self.packet_handler.getTxRxResult(dxl_comm_result)}'
                        )
                    elif dxl_error != 0:
                        self.get_logger().error(
                            f'Error: {self.packet_handler.getRxPacketError(dxl_error)}'
                        )
                    else:
                        success = 3  # Going to use 3 to mean goal was in deadzone

            case MotorMode.POSITION:
                dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, self.id, ADDR_GOAL_POSITION, goal
                )
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error(
                        f'Position Set Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
                    )
                elif dxl_error != 0:
                    self.get_logger().error(
                        f'Position Set Error: {self.packet_handler.getRxPacketError(dxl_error)}'
                    )
                else:
                    success = True

            case MotorMode.EXT_POSITION:
                dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, self.id, ADDR_GOAL_POSITION, goal
                )
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error(
                        f'Position Set Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
                    )
                elif dxl_error != 0:
                    self.get_logger().error(
                        f'Position Set Error: {self.packet_handler.getRxPacketError(dxl_error)}'
                    )
                else:
                    success = True
            case MotorMode.PWM:
                pass

        match success:
            case True:
                self.get_logger().info(
                    f'Motor {self.id}: Successfully set {self.mode.name} goal to {goal}'
                )
            case False:
                self.get_logger().info(
                    f'Motor {self.id}: Failed to set {self.mode.name} goal to {goal}'
                )
            case 3:
                self.get_logger().info(
                    f'Motor {self.id}: Deadzone goal {goal} (mode: {self.mode.name})'
                )

    def switch_mode_cb(self, request, response):
        """
        Switch the control mode of this node's motor.

        Args:
        ----
        request  (polydact_interfaces/srv/Mode):
                request.mode contains value to set as the motor's control mode.
        response (polydact_interfaces/srv/Mode):
                reponse.success is True if the control mode is successfully changed

        """
        response.success = True

        # Filter out invalid values
        if request.mode not in [1, 3, 4, 16]:
            self.get_logger().info(
                f'{request.mode} not valid Control Mode. \
                    Use 1 (vel), 3 (pos), 4 (ext pos), or 16 (PWM)'
            )
            response.success = False
        # Turn torque off to that the control mode can be changed
        if response.success:
            response.success = self.toggle_on_off(-1)
        # Change the control mode
        if response.success:
            self.get_logger().info(f'Setting mode: {self.mode.name}')
            response.success = self.set_mode(request.mode)
        # Turn torque back on and change state if mode was successfully changed
        if response.success:
            self.mode = MotorMode(request.mode)
            response.success = self.toggle_on_off(1)

        return response

    def toggle_on_off(self, new_state=0) -> bool:
        """
        Toggle the motor on or off.

        Args:
        ----
        new_state (int): 0 (default) toggles current state, -1 turns motor off, 1 turns motor on

        Returns
        -------
        (bool): True if the motor state was succesfully toggled_

        """
        success = False
        self.get_logger().debug(f'Motor on/off was {self.active}.')
        if (self.active and new_state == 1) or (not self.active and new_state == -1):
            self.get_logger().debug('Toggle request matches current state')
            success = True
        else:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                self.id,
                ADDR_TORQUE_ENABLE,
                int(not self.active),
            )

            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error('Was not able to toggle motor on/off')

            else:
                success = True
                self.active = not self.active
                self.get_logger().debug('Was able to toggle motor on/off')
            self.get_logger().debug(f'Torque on/off is now {self.active}.')
        return success

    def set_mode(self, mode: int) -> bool:
        """
        Set the control mode of the motor.

        Args:
        ----
        mode (int): 1, 3, 4, or 16 to set mode to velocity, position, extended position, or PWM

        Returns
        -------
        (bool): True if the state was correctly set to the selected mode.

        """
        success = False
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id, ADDR_OPERATING_MODE, mode
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f'Failed to set Control Mode {mode}: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        else:
            success = True
            self.get_logger().debug(f'Succeeded to set Control Mode {mode}.')
        return success

    def timer_callback(self):
        """Read and publish motor position, velocity, and load."""
        dxl_present_position = False
        dxl_present_velocity = False
        dxl_present_load = False

        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.id, ADDR_PRESENT_POSITION
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f'Position Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            self.get_logger().error(
                f'Position Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )

        dxl_present_velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.id, ADDR_PRESENT_VELOCITY
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f'Velocity Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            self.get_logger().error(
                f'Velocity Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )

        dxl_present_load, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.id, ADDR_PRESENT_LOAD
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f'Load Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            self.get_logger().error(
                f'Load Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )

        # self.get_logger().debug(
        #    f'\nPos: {dxl_present_position}\nVel: {dxl_present_velocity}\nLoa: {dxl_present_load}'
        # )
        self.get_logger().debug(f'Vel: {dxl_present_velocity}')
        self.posi_pub.publish(Int16(data=dxl_present_position))
        self.velo_pub.publish(Int16(data=dxl_present_velocity))
        self.load_pub.publish(Int16(data=dxl_present_load))

    def __del__(self):
        """Unknown, honestly; it was in the example code."""
        self.packet_handler.write1ByteTxRx(
            self.port_handler, 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        self.port_handler.closePort()
        self.get_logger().info('Shutting down single_motor')


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = SingleMotor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
