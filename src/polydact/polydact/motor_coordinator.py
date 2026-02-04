"""Test different control modes of one Dynamixel motor."""

from enum import Enum

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
from polydact_interfaces.msg import MotorState
from polydact_interfaces.msg import MotorStateArray
from polydact_interfaces.srv import Mode
import rclpy
from rclpy.node import Node
import serial

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


class MotorCoordinator(Node):
    """
    Handler for Dynamixel motors.

    Publishers:
        + "position" (std_msgs/msg/Int16) - The current position of the motor
        + "velocity" (std_msgs/msg/Int16) - The current velocity of the motor
        + "load" (std_msgs/msg/Int16) - The current load of the motor

    Subscribers:
        + "motor_goal" (polydact_interfaces/msg/MotorState) - A goal to set a particular motor to

    Services:
        + "set_mode" (polydact_interfaces/srv/Mode) - What Control Mode to set the motor to.
            ( 1 (vel), 3 (pos), 4 (ext pos), or 16 (PWM)')

    Parameters
    ----------
        + "motor_ids" - The motor ID to to control with this node
        + "control_mode" - The initial Control Mode to use

    """

    def __init__(self):
        """Initialize motor control."""
        super().__init__('motor_coordiantor')
        # Handle motor port communification
        self.port_handler = PortHandler(DEVICE_NAME)

        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        while True:
            try:
                self.port_handler.openPort()
                break
            except FileNotFoundError:
                self.get_logger().error(
                    f'Could not open port {DEVICE_NAME}. Is motor board in the correct port',
                    throttle_duration_sec=1,
                )
            except serial.SerialException:
                self.get_logger().error(
                    f'Could not open port {DEVICE_NAME}. Is motor board in the correct port',
                    throttle_duration_sec=1,
                )

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('Baudrate setting failure')
            return
        self.get_logger().info('Baudrate set')

        # Set motors to my settings
        self.declare_parameter('motor_ids', [2, 3, 5])
        self.ids = self.get_parameter('motor_ids').value

        self.declare_parameter('Control_Mode', 1)
        self.mode = MotorMode(self.get_parameter('Control_Mode').value)

        for id in self.ids:
            self.toggle_on_off(id, 0)
            self.set_mode(id, self.mode.value)
            self.toggle_on_off(id, 1)

        self.addresses = {
            'toggle_mode': 11,
            'toggle_torque': 64,
            MotorMode.POSITION: 116,
            MotorMode.VELOCITY: 104,
            'get_pos': 132,
            'get_load': 126,
            'get_vel': 128,
        }

        self.goal_array_sub = self.create_subscription(
            MotorStateArray, 'motor_goal_array', self.goal_array_cb, 10
        )

        self.goal_sub = self.create_subscription(
            MotorState, 'motor_goal', self.set_single_goal, 10
        )
        self.mode_srv = self.create_service(Mode, 'set_mode', self.switch_mode_cb)

        self.posi_pub = self.create_publisher(MotorState, 'cur_position', 10)
        self.velo_pub = self.create_publisher(MotorState, 'cur_velocity', 10)
        self.load_pub = self.create_publisher(MotorState, 'cur_load', 10)

        self.timer = self.create_timer(1 / 100, self.timer_callback)

    def goal_array_cb(self, state_array: MotorStateArray):
        """
        Set the goals for all motors contained in the MotorStateArray.

        Args:
        ----
        state_array (polydact_interfaces/msg/MotorStateArray): The MotorStates (id, state) to set

        """
        self.get_logger().debug(
            f'MotorStateArray received (len {len(state_array.states)} states) '
        )
        for state_msg in state_array.states:
            self.set_single_goal(state_msg)

    def set_single_goal(self, msg: MotorState):
        """
        Set the position, velocity, or PWM goal for the motor.

        Args:
        ----
        msg (polydact_interfaces/msg/MotorState): msg.id is the motor to set the goal for
                                            msg.state is the goal to set to

        """
        self.get_logger().debug(
            f'MotorState {msg.id} {msg.state} received (current mode: {self.mode.name})'
        )
        if msg.id not in self.ids:
            self.get_logger().error(f'Unexpected motor goal received: {msg}')
            return

        id = msg.id
        goal = -1 * msg.state
        success = False
        match self.mode:
            case MotorMode.VELOCITY:
                deadzone = 0.3
                if abs(goal) > deadzone:
                    self.get_logger().debug(f'goal pre change{goal}')
                    goal = int(goal / 10 * 300)
                    self.get_logger().debug(f'goal post change{goal}')
                    dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                        self.port_handler, id, ADDR_GOAL_VELOCITY, goal
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
                        self.port_handler, id, ADDR_GOAL_VELOCITY, goal
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
                    self.port_handler, id, ADDR_GOAL_POSITION, goal
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
                    self.port_handler, id, ADDR_GOAL_POSITION, goal
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
                self.get_logger().debug(
                    f'Motor {id}: Successfully set {self.mode.name} goal to {goal}'
                )
            case False:
                self.get_logger().debug(
                    f'Motor {id}: Failed to set {self.mode.name} goal to {goal}'
                )
            case 3:
                self.get_logger().debug(
                    f'Motor {id}: Deadzone goal {goal} (mode: {self.mode.name})'
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
        if request.mode not in [0, 1, 3, 4, 16]:
            self.get_logger().info(
                f'{request.mode} not valid Control Mode. \
                    Use 1 (vel), 3 (pos), 4 (ext pos), or 16 (PWM)'
            )
            response.success = False
        self.get_logger().info(f'ids:{self.ids}')
        for id in self.ids:
            self.get_logger().info(f'success: {response.success}')
            # Turn torque off to that the control mode can be changed
            if response.success:
                self.get_logger().info(f'id: {id}')
                response.success = self.toggle_on_off(id, 0)
            # Change the control mode
            if response.success and request.mode != 0:
                self.get_logger().info(f'Setting mode: {self.mode.name}')
                response.success = self.set_mode(id, request.mode)
            # Turn torque back on and change state if mode was successfully changed
            if response.success and request.mode != 0:
                self.mode = MotorMode(request.mode)
                response.success = self.toggle_on_off(id, 1)

        return response

    def toggle_on_off(self, id, new_state=0) -> bool:
        """
        Toggle a motor on or off.

        Args:
        ----
        id (int): The motor to toggle
        new_state (int): 0 (default) toggles current state, -1 turns motor off, 1 turns motor on

        Returns
        -------
        (bool): True if the motor state was succesfully toggled.

        """
        success = False
        # self.get_logger().debug(f'Motor on/off was {self.active}.')
        if False:  # (self.active and new_state == 1) or (not self.active and new_state == -1):
            self.get_logger().debug('Toggle request matches current state')
            success = True
        else:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                id,
                ADDR_TORQUE_ENABLE,
                new_state,
            )

            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(
                    f'Was not able to toggle Motor {id}. on/off 1/-1 : {new_state}'
                )

            else:
                success = True
                # self.active = not self.active
                self.get_logger().debug(
                    f'Was able to toggle motor on/off. on/off 1/-1 : {new_state}'
                )
            # self.get_logger().info(f'Torque on/off is now {self.active}.')
        return success

    def set_mode(self, id: int, mode: int) -> bool:
        """
        Set the control mode of the motor.

        Args:
        ----
        id (int): The id of the motor to change the mode of
        mode (int): 1, 3, 4, or 16 to set mode to velocity, position, extended position, or PWM

        Returns
        -------
        (bool): True if the state was correctly set to the selected mode.

        """
        success = False
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, id, ADDR_OPERATING_MODE, mode
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

    def get_state(self, id: int):
        """
        Get the state of one motor.

        Args:
        ----
        id (int): The motor to read and publish the state of

        Return:
        ------
        int, int, int: the current position, velocity, and load of the specified motor

        """
        current_position = False
        current_velocity = False
        current_load = False

        current_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, id, ADDR_PRESENT_POSITION
        )

        if dxl_comm_result != COMM_SUCCESS:
            current_position = -1000
            self.get_logger().error(
                f'Position Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            current_position = -1000
            self.get_logger().error(
                f'Position Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )

        current_velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, id, ADDR_PRESENT_VELOCITY
        )
        if dxl_comm_result != COMM_SUCCESS:
            current_velocity = -1000
            self.get_logger().error(
                f'Velocity Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            current_position = -1000
            self.get_logger().error(
                f'Velocity Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )

        current_load, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, id, ADDR_PRESENT_LOAD
        )
        if dxl_comm_result != COMM_SUCCESS:
            current_load = -1000
            self.get_logger().error(
                f'Load Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            current_position = -1000
            self.get_logger().error(
                f'Load Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )
        self.get_logger().debug(
            f'{id}: {float(current_position)}, {float(current_velocity)}, {float(current_load)}'
        )
        return float(current_position), float(current_velocity), float(current_load)

    def pub_state(self, id: int, state: tuple):
        """
        Publish the current position, velocity, and load of a specific motor.

        Args:
        ----
        id (int): Which motor's state is being published?
        state (tuple): current (position, velocity, load) of the motor

        """
        if -1000 in state:
            self.get_logger().error('Faulty state published (bad values published as -1000)')
        else:
            self.posi_pub.publish(MotorState(id=id, state=state[0]))
            self.velo_pub.publish(MotorState(id=id, state=state[1]))
            self.load_pub.publish(MotorState(id=id, state=state[2]))

    def timer_callback(self):
        """Read and publish motor position, velocity, and load."""
        for id in self.ids:
            self.pub_state(id, self.get_state(id))
        # dxl_present_position = False
        # dxl_present_velocity = False
        # dxl_present_load = False

        # dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
        #     self.port_handler, self.ids[0], ADDR_PRESENT_POSITION
        # )

        # if dxl_comm_result != COMM_SUCCESS:
        #     self.get_logger().error(
        #         f'Position Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
        #     )
        # elif dxl_error != 0:
        #     self.get_logger().error(
        #         f'Position Error: {self.packet_handler.getRxPacketError(dxl_error)}'
        #     )

        # dxl_present_velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
        #     self.port_handler, self.ids[0], ADDR_PRESENT_VELOCITY
        # )
        # if dxl_comm_result != COMM_SUCCESS:
        #     self.get_logger().error(
        #         f'Velocity Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
        #     )
        # elif dxl_error != 0:
        #     self.get_logger().error(
        #         f'Velocity Error: {self.packet_handler.getRxPacketError(dxl_error)}'
        #     )

        # dxl_present_load, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
        #     self.port_handler, self.ids[0], ADDR_PRESENT_LOAD
        # )
        # if dxl_comm_result != COMM_SUCCESS:
        #     self.get_logger().error(
        #         f'Load Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
        #     )
        # elif dxl_error != 0:
        #     self.get_logger().error(
        #         f'Load Error: {self.packet_handler.getRxPacketError(dxl_error)}'
        #     )

        # # self.get_logger().debug(

    # #    f'\nPos: {dxl_present_position}\nVel: {dxl_present_velocity}\nLoa: {dxl_present_load}'
    # # )
    # for id in self.ids:

    #     self.posi_pub.publish(MotorState(state=dxl_present_position))
    #     self.velo_pub.publish(MotorState(state=dxl_present_velocity))
    #     self.load_pub.publish(MotorState(state=dxl_present_load))

    def __del__(self):
        """Turn off the motors and close port if the node is closed properly."""
        if self.ids:
            for id in self.ids:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
                )
        self.port_handler.closePort()
        self.get_logger().info('Shutting down motors')


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = MotorCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
