"""Reading Polydact's flex sensors."""


class Sensor:
    """Sensor class holds values for each individual sensor."""

    def __init__(self, sensor_id, motor_id):
        """
        Initialize the sensor.

        Args:
        ----
        sensor_id (int): Which sensor is this? My finger map is 1 = ring, 2 = middle, 3 = index.
        motor_id (int): What is the Dynamixel ID of the motor that this sensor will control?

        """
        self.sensor_id = sensor_id
        self.motor_id = motor_id
        self.min = 10000.0
        self.max = 0.0
        self.selector_min = 0.0
        self.selector_max = 0.0
        self.smoothing = 1
        self.reads = [0.0] * self.smoothing
        self.calibrated = 0

    def new_read(self, value: float):
        """
        Add the newest raw sensor reading to the rolling list.

        Args:
        ----
        value (float): The new reading to add to the rolling list.

        """
        self.reads.append(value)
        self.reads.pop(0)

    def get_value(self) -> float:
        """
        Get the normalized rolling average of sensor readings.

        Returns
        -------
         (float): The normalized rolling average of sensor readings. If sensor is uncalibrated,
                  0.0 will be returned (freezes the motors).

        """
        if not self.calibrated:
            return float(0.0)
        else:
            value = sum(self.reads) / self.smoothing  # Average
            value -= (self.max + self.min) / 2  # Center at 0
            value /= (self.max - self.min) / 2  # Squash the values down to -1 to 1

            # If the max and min are off for some reason, clamp the values
            if value > 1:
                value = 1
            elif value < -1:
                value = -1
            return float(value)

    def fill_average(self):
        """Fill the reads list with a neutral value so motors initialize stopped."""
        self.reads = [(self.max + self.min) / 2] * self.smoothing

    def reset(self):
        """Clear the calibrated values, but keep the motor id assignment."""
        self.min = 10000
        self.max = 0.0
        self.reads = [0.0] * self.smoothing
        self.calibrated = 0
