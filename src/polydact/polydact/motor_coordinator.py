"""Test different control modes of one Dynamixel motor."""

from polydact.dynamixel_interface import Motor, DynamixelInterface
from polydact_interfaces.msg import MotorState

from polydact_interfaces.srv import Mode
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class MotorCoordinator(Node):
    """
    MotorCoordinator is the intermediary between the control device (glove) and the motors.

    Publishers:
        + "motor_states" (sensor_msgs/msg/JointState) - Actual states of the motors

    Subscribers:
        + "motor_goal" (polydact_interfaces/msg/MotorState) - A goal to set a particular motor to

    Services:
        + "set_mode" (polydact_interfaces/srv/Mode) - What Control Mode to set the motor to.
            (0 (off), 1 (vel))

    Parameters
    ----------
        + "motor_ids" - The motor ID to to control with this node
        + "control_mode" - The initial Control Mode to use

    """

    def __init__(self):
        """Initialize motor control."""
        super().__init__('motor_coordiantor')
        self.active = 0

        self.dyn = DynamixelInterface(self)

        # Set motors to my settings
        self.declare_parameter('motor_ids', [1, 2, 3])
        self.ids = self.get_parameter('motor_ids').value

        self.declare_parameter('Control_Mode', 1)
        self.mode = self.get_parameter('Control_Mode').value

        self.motors = {}
        for id in self.ids:
            self.motors.update({id: Motor(self.dyn, id)})

        # Motors should initialize off, but loop through just to be sure
        # Should probably ad a check to make sure they're actually all off
        for motor in self.motors.values():
            motor.set_off(0)

        self.goal_sub = self.create_subscription(
            MotorState,
            'motor_goal',
            self.set_single_goal,
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.mode_srv = self.create_service(Mode, 'set_mode', self.switch_mode_cb)

        self.motor_states_pub = self.create_publisher(JointState, 'motor_states', 10)

        self.posi_pub = self.create_publisher(MotorState, 'cur_position', 10)
        self.velo_pub = self.create_publisher(MotorState, 'cur_velocity', 10)
        self.load_pub = self.create_publisher(MotorState, 'cur_load', 10)

        self.timer = self.create_timer(1 / 100, self.timer_callback)
        self.get_logger().info('Motor Coordinator ready')

    def set_single_goal(self, msg: MotorState):
        """
        Set the position, velocity, or PWM goal for a single motor.

        Args:
        ----
        msg (polydact_interfaces/msg/MotorState): msg.id is the motor to set the goal for
                                                  msg.state is the goal to set to

        """
        self.get_logger().debug(f'MotorState {msg.id} {msg.state} received.)')
        if msg.id not in self.motors.keys():
            self.get_logger().error(f'Unexpected motor id received: {msg}')
            return

        deadzone = 0.3
        self.motors[msg.id].set_velocity(-1 * msg.state, deadzone)

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
        if request.mode not in [0, 1]:
            self.get_logger().error(
                f'{request.mode} not valid Control Mode. Use 0 (off), 1 (velocity)'
            )
            response.success = False

        for motor in self.motors.values():
            motor.set_mode(request.mode)

        return response

    def timer_callback(self):
        """Read and publish motor position, velocity, and load."""
        motor_states = JointState(header=Header(stamp=self.get_clock().now().to_msg()))

        for motor in self.motors.values():
            motor.get_state()
            # TODO: Add some sort of checking for if the get_state fails
            motor_states.name.append(str(motor.id))
            motor_states.position.append(motor.position)
            motor_states.velocity.append(motor.velocity)
            motor_states.effort.append(motor.effort)

        self.motor_states_pub.publish(motor_states)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = MotorCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
