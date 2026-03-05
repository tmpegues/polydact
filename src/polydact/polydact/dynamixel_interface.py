"""Interface with Dynmaixel SDK."""

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler

import serial


# Control table address
ADDR_OPERATING_MODE = 11  # Control table address is different in Dynamixel model
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_PWM = 124
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128


# Protocol version
PROTOCOL_VERSION = 2.0  # Default Protocol version of DYNAMIXEL X series.

# Default settings
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICE_NAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque


class DynamixelInterface:
    """Hold the Dynamixel port handler, packet handler, and others."""

    def __init__(self, node):
        """
        Initialize Dynamixel SDK port and packet handler.

        Args:
        ----
        node (rclpy.Node): The node that this object is being used in. Used for logging.

        """
        # Handle motor port communification
        self.node = node
        self.port_handler = PortHandler(DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        while True:
            try:
                self.port_handler.openPort()
                break
            except FileNotFoundError:
                self.node.get_logger().error(
                    f'Dyn: Could not open port {DEVICE_NAME}. Is motor board in the correct port?',
                    throttle_duration_sec=1,
                )
            except serial.SerialException:
                self.node.get_logger().error(
                    f'Dyn: Could not open port {DEVICE_NAME}. Is motor board in the correct port?',
                    throttle_duration_sec=1,
                )

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.node.get_logger().error('Dyn: Could not set baudrate')
            return
        self.node.get_logger().debug('Dyn: Baudrate set')

        self.addresses = {
            'toggle_mode': 11,
            'toggle_torque': 64,
            'set_position': 116,
            'set_velocity': 104,
            'get_pos': 132,
            'get_load': 126,
            'get_vel': 128,
        }

    def send_velocity(self, motor_id: int, goal: float):
        """Write the velocity to the motor."""
        success = False
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ADDR_GOAL_VELOCITY, goal
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.node.get_logger().error(
                f'Dyn Error: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            self.node.get_logger().error(
                f'Dyn Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )
        else:
            success = True

        match success:
            case True:
                self.node.get_logger().debug(
                    f'Motor {motor_id}: Successfully set velocity goal to {goal}'
                )
            case False:
                self.node.get_logger().debug(
                    f'Motor {motor_id}: Failed to set velocity goal to {goal}'
                )

    def send_on_off(self, motor_id: int, new_state: int = 0):
        """Write the active/inactive state to the motor."""
        success = False

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            motor_id,
            ADDR_TORQUE_ENABLE,
            new_state,
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.node.get_logger().error(
                f'Dyn: Was not able to toggle Motor {motor_id}. on/off 1/0: {new_state}'
            )

        else:
            success = True
            self.node.get_logger().debug(
                f'Dyn: Was able to toggle motor on/off. on/off 1/0: {new_state}'
            )

        return success

    def send_mode(self, motor_id: int, mode: int) -> bool:
        """
        Set the control mode of the motor.

        Args:
        ----
        motor_id (int): The motor_id of the motor to change the mode of
        mode (int): 1, 3, 4, or 16 to set mode to velocity, position, extended position, or PWM

        Returns
        -------
        (bool): True if the state was correctly set to the selected mode.

        """
        success = False
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_OPERATING_MODE, mode
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.node.get_logger().error(
                f'Failed to set Control Mode {mode}: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        else:
            success = True
            self.node.get_logger().debug(f'Succeeded to set Control Mode {mode}.')
        return success

    def read_position(self, motor_id: int) -> int:
        """
        Read the position of a specific motor.

        Args:
        ----
        motor_id (int): The motor_id of the motor to read from.

        Returns
        -------
         (int): The current position of the motor

        """
        current_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, ADDR_PRESENT_POSITION
        )

        if dxl_comm_result != COMM_SUCCESS:
            current_position = False
            self.node.get_logger().error(
                f'Position Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            current_position = False
            self.node.get_logger().error(
                f'Position Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )
        self.node.get_logger().debug(f'DYN: Motor {motor_id} position {current_position}')
        return current_position

    def read_velocity(self, motor_id):
        """
        Read the velocity of a specific motor.

        Args:
        ----
        motor_id (int): The motor_id of the motor to read from.

        Returns
        -------
         (int): The current velocity of the motor

        """
        current_velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, ADDR_PRESENT_VELOCITY
        )

        if dxl_comm_result != COMM_SUCCESS:
            current_velocity = False
            self.node.get_logger().error(
                f'Velocity Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            current_velocity = False
            self.node.get_logger().error(
                f'Velocity Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )
        return current_velocity

    def read_effort(self, motor_id):
        """
        Read the effort of a specific motor.

        Args:
        ----
        motor_id (int): The motor_id of the motor to read from.

        Returns
        -------
         (int): The current effort of the motor

        """
        current_effort, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, ADDR_PRESENT_CURRENT
        )
        if dxl_comm_result != COMM_SUCCESS:
            current_effort = False
            self.node.get_logger().error(
                f'Load Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}'
            )
        elif dxl_error != 0:
            current_effort = False
            self.node.get_logger().error(
                f'Load Error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )
        self.node.get_logger().debug(f'DYN: Motor {motor_id} current {current_effort}')
        current_effort, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, ADDR_PRESENT_PWM
        )
        self.node.get_logger().debug(f'DYN: Motor {motor_id} pwm {current_effort}')

        return current_effort


class Motor:
    """Polydact motor class using Dynmaixel SDK."""

    def __init__(self, interface: DynamixelInterface, motor_id: int):
        """
        Initialize a single motor.

        Args:
        ----
        interface (DynamixelInterface): Contains the port and packet handler that will control this
                                        motor.
        motor_id (int): The Dynamixel motor ID.

        """
        self.motor_id = motor_id
        self.dyn = interface
        self.position = 0
        self.velocity = 0
        self.effort = 0
        # Initialize off
        self.dyn.send_on_off(self.motor_id, 0)
        self.active = 0

    def set_velocity(self, goal: float, deadzone: float):
        """Set this motor's velocity to the proportional goal received here."""
        if abs(goal) > deadzone:
            if goal > 0:
                goal = (goal - deadzone) / (1 - deadzone)
            if goal < 0:
                goal = (goal + deadzone) / (1 - deadzone)
            goal = int(goal**3 * 300)
        else:
            goal = 0
        success = self.dyn.send_velocity(self.motor_id, goal)

        match success:
            case True:
                self.dyn.node.get_logger().debug(
                    f'Motor {self.motor_id}: Successfully set {self.mode} goal to {goal}'
                )
            case False:
                self.dyn.node.get_logger().debug(
                    f'Motor {self.motor_id}: Failed to set {self.mode} goal to {goal}'
                )

    def set_off(self):
        """
        Turn the motor off.

        No specific function is given to turn them back on. To turn motors back on, set the mode.

        Returns
        -------
        (bool): Was the motor succesfully turned off?

        """
        success = self.dyn.send_on_off(self.motor_id, 0)
        if success:
            self.active = 0
        return success

    def set_mode(self, mode: int):
        """
        Set the control mode of the motor.

        Only velocity (mode 1) is usable.
        """
        success = self.set_off()
        # Change the control mode
        if success and mode != 0:
            success = self.dyn.send_mode(self.motor_id, mode)
        # Turn torque back on and change state if mode was successfully changed
        if success and mode != 0:
            self.mode = mode
            success = self.dyn.send_on_off(self.motor_id, 1)
        if success:
            self.active = 1

    def get_state(self):
        """Read the current position, velocity, and effort of the motor."""
        self.position = self.dyn.read_position(self.motor_id)
        self.velocity = self.dyn.read_velocity(self.motor_id)
        self.effort = self.dyn.read_effort(self.motor_id)

        self.dyn.node.get_logger().debug(f'Motor {self.motor_id} position {self.position}')
        self.dyn.node.get_logger().debug(f'Motor {self.motor_id} velocity  {self.velocity}')
        self.dyn.node.get_logger().debug(f'Motor {self.motor_id} effort  {self.effort}')
