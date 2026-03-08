"""Read the serial monitor from the Pico 2 to publish a motor goal."""

from polydact_interfaces.msg import MotorGoal
from polydact_interfaces.srv import Mode
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_srvs.srv import Empty
from polydact.sensor import Sensor

from gpiozero import MCP3008


class PiADCSensorNode(Node):
    """Reads the MCP3008 ADC for sensors and publishes motor goals."""

    def __init__(self):
        """Initialize sensors and goal publisher."""
        super().__init__('pi_adc_sensor_node')

        # Creating publisher first to that PlotJuggler doesn't have to wait for calibration
        self.goal_pub = self.create_publisher(
            MotorGoal,
            'motor_goal',
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.remap_srv = self.create_service(Empty, 'remap', self.remap_cb)

        # Get motor list, initialize sensors, initialize ADC
        self.declare_parameter('sensor_ids', [1, 2, 3])
        sensor_ids = self.get_parameter('sensor_ids').value

        self.declare_parameter('motor_ids', [1, 2, 3])
        self.motor_ids = self.get_parameter('motor_ids').value

        self.adc = {channel: MCP3008(channel=channel) for channel in sensor_ids}

        self.sensors = {
            sensor_id: Sensor(sensor_id, motor_id)
            for sensor_id, motor_id in zip(sensor_ids, self.motor_ids)
        }
        # for sensor_id, motor_id in zip(sensor_ids, self.motor_ids):
        #     self.sensors.update({sensor_id: Sensor(sensor_id, motor_id)})

        self.set_mode_client = self.create_client(Mode, 'set_mode')
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Failed to find Mode ("set_mode") service')
            raise RuntimeError('Failed to find Mode ("set_mode") service')

        # Calibrate sensor min/max with specified number of readings from each sensor
        self.min_max_calibration(500)

        # Begin publishing normalized sensor readings
        pub_freq = 100
        read_freq = 100
        self.last_published = 0

        self.reading_timer = self.create_timer(1 / read_freq, self.reading_timer_callback)
        self.publishing_timer = self.create_timer(
            int(1 / pub_freq), self.publishing_timer_callback
        )
        self.get_logger().info('Pi ADC Reader fully initialized')

    def full_calibration(self, num_points: int):
        """
        Calibrate the glove by finding min and max flexes for all fingers, then remap to motors.

        Args:
        ----
        num_points (int): How many points are to be collected for the min max calibration?

        """
        self.freeze(True)

        self.min_max_calibration(num_points)
        self.map_sensor_to_motor()

        self.freeze(False)

    def freeze(self, freeze_on: bool, read: bool = False):
        """
        Stop all motors by pausing timer, then sending Goal = 0.

        Saving of sensor values can also be paused optionally, but unfreezing will always restart
        sensor saving.

        Args:
        ----
        freeze_on (bool): True will stop motors, False will turn velocity control back on
        read (bool): True will also top lines from being saved, which is used for remapping.

        """
        match freeze_on:
            case True:
                if read:
                    self.reading_timer.cancel()
                self.publishing_timer.cancel()
                for motor_id in self.motor_ids:
                    self.goal_pub.publish(MotorGoal(motor_id=motor_id, goal=0))

            case False:
                for sensor in self.sensors.values():
                    sensor.fill_average()
                self.publishing_timer.reset()
                self.reading_timer.reset()

    def get_max_diff(self, reads: int = 100) -> int:
        """
        Get the ID of the sensor that gets bent the most over the next selected no of reads.

        Args:
        ----
        reads (int): How many reads to make. Not how many per sensor.

        Returns
        -------
         (int): The ID of the sensor that experienced the largest bend. -1 means there was an issue

        """
        # Clear the saved selector values
        for sensor in self.sensors.values():
            sensor.selector_min = 10000
            sensor.selector_max = -10000

        i = 0
        # Read the selected total number of lines
        while i <= reads:
            for id, value in self.read_adc().items():
                if id not in self.sensors.keys():
                    self.get_logger().error(f'Sensor message id {id} is not valid id.')
                    break
                sensor = self.sensors[id]
                if value > sensor.selector_max:
                    sensor.selector_max = value
                elif value < sensor.selector_min:
                    sensor.selector_min = value
                else:
                    self.get_logger().error(
                        f'Unexpected condition in sensor remapping. id, value: {id}, {value}'
                    )

        # Now that lines are read, determine which sensor has been flexed the most
        diff = 0
        max_bent = -1
        for id, sensor in self.sensors.items():
            if abs(sensor.selector_max - sensor.selector_min) > diff:
                diff = sensor.selector_max - sensor.selector_min
                max_bent = id

        return max_bent

    def remap_cb(self, request, response):
        """
        Remap sensors.

        Args:
        ----
        request (std_srv/srv/Empty): unused
        response (std_srv/srv/Empty): unused

        """
        self.map_sensor_to_motor()

    def map_sensor_to_motor(self):
        """Change the assignment of which sensor controls which motor."""
        self.freeze(True, True)
        retry = True
        while retry:
            retry = False
            # Clear motor ids for all sensors
            for sensor in self.sensors.values():
                sensor.motor_id = -1

            # Cycle through motors again, setting sensors as we go
            for motor in self.motor_ids:
                self.get_logger().info(f'Motor: {motor}')
                while True:
                    self.get_logger().info(f'Wiggle the sensor to use with motor {motor}')
                    # Read for 100 values, find max bent sensor
                    sensor1 = self.get_max_diff(100)
                    self.get_logger().info(f'Sensor {sensor1} selected for {motor}')
                    self.get_logger().info(f'Keep wiggling sensor {sensor1} to confirm selection')
                    # Read another 100, check for match
                    sensor2 = self.get_max_diff(100)
                    # If the sensors match and that sensor doesn't have a motor saved yet, save
                    if sensor1 == sensor2 and self.sensors[sensor1].motor_id == -1:
                        self.sensors[sensor1].motor_id = motor
                        self.get_logger().info(f'Saving: Sensor {sensor1} maps to motor {motor}')
                        break
                    elif sensor1 != sensor2:
                        self.get_logger().error(
                            f'1st sensor {sensor1} and 2nd sensor {sensor2} do not match.'
                        )
                    elif self.sensors[sensor1].motor_id != -1:
                        self.get_logger().error(
                            f'Sensor {sensor1} already assigned: {self.sensors[sensor1].motor_id}.'
                        )
                    self.get_logger().info(f'Retrying. Motor = {motor}.')

            # Check that each sensor has a motor (id != -1) and that each motor is unique
            motor_list = self.motor_ids.copy()
            for s_id, sensor in self.sensors.items():
                if sensor.motor_id in motor_list:  # This case is what we want
                    self.get_logger().info(f'Sensor {s_id} is assigned to motor {sensor.motor_id}')
                    motor_list.remove(sensor.motor_id)
                elif sensor.motor_id == -1:
                    retry = True
                    self.get_logger().error(f'Sensor {s_id} was not assigned a motor')
                    break
                elif sensor.motor_id not in motor_list:
                    retry = True
                    self.get_logger().error('This motor is already  assigned or is invalid ID.')
                    break
                else:
                    self.get_logger().error(f'Unexpected case. S: {s_id}, M: {sensor.motor_id}')

            # Check that each motor was assigned
            if len(motor_list) != 0:
                retry = True
                self.get_logger().error(f'Motors unassigned: {motor_list}.')

            if retry:
                self.get_logger().info('Retrying remapping.')
            else:
                self.get_logger().info('Remapping complete:')
                for sensor in self.sensors.values():
                    self.get_logger().info(f'Sensor {sensor.sensor_id} -> Motor {sensor.motor_id}')
                self.freeze(False)

    def min_max_calibration(self, num_points: int):
        """
        Read values from the connected sensors for a few seconds to determine ranges.

        Args:
        ----
        num_points (int): How many points will be checked to figure out each sensor's min/max range

        """
        calibrated = 0
        loop = 0
        self.get_logger().info(f'Beginning min/max calibration for {len(self.sensors)} sensors.')
        while calibrated < len(self.sensors) * num_points:
            self.get_logger().info(
                f'Loop {loop} ({calibrated}/{len(self.sensors) * num_points})',
                throttle_duration_sec=1,
            )
            loop += 1
            reads = self.read_adc()
            for id, value in reads.items():
                if id not in self.sensors:
                    self.get_logger().error(f'Sensor message id {id} is not valid id.')
                    break
                sensor = self.sensors[id]
                if sensor.calibrated >= num_points:
                    # If we've already read many points, do not process any further points
                    pass
                elif sensor.calibrated < num_points:
                    # If we haven't read this many points, check against min and max
                    sensor.calibrated += 1
                    calibrated += 1
                    if value > sensor.max:
                        sensor.max = value
                    elif value < sensor.min:
                        sensor.min = value
                    if sensor.calibrated >= num_points:
                        self.get_logger().info(
                            f'Sensor {sensor.sensor_id} has all {num_points} min/max points read.'
                        )
                        sensor.fill_average()
                else:
                    self.get_logger().error(
                        f'Unexpected condition in calibration. id, value: {id}, {value}'
                    )
        self.get_logger().info('Sensor min/max calibration complete:')
        for id, sensor in self.sensors.items():
            self.get_logger().info(f'Sensor {sensor.sensor_id}: {sensor.min} - {sensor.max}')

    def read_adc(self) -> dict:
        """
        Get values for all 3 sensors on the ADC.

        Returns
        -------
        (dict): {ADC channel (int): value (float)}

        """
        return {channel: self.adc[channel].value for channel in self.adc.keys()}

    def reading_timer_callback(self):
        """Read the adc monitor."""
        for id, value in self.read_adc().items():
            if id in self.sensors.keys():
                self.sensors[id].new_read(value)

    def publishing_timer_callback(self):
        """Publish all motor goals individually."""
        self.get_logger().debug('Publishing Timer')

        self.goal_pub.publish(
            MotorGoal(
                motor_id=self.sensors[list(self.sensors.keys())[self.last_published]].motor_id,
                goal=self.sensors[list(self.sensors.keys())[self.last_published]].get_value(),
            )
        )
        self.last_published += 1
        self.last_published %= len(self.sensors)


def main(args=None):
    """Run the pi_adc_sensor_node."""
    rclpy.init(args=args)
    node = PiADCSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
