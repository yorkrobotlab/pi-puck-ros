#!/usr/bin/env python
"""ROS Node to expose a topics for the IMU (LSM9DS1) on the Pi-puck."""

# Base Imports
import math
from os import path
from json import load
from threading import Thread

# ROS imports
import rospy
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu, Temperature, MagneticField

# Local imports
from lsm9ds1 import LSM9DS1  # pylint: disable=relative-import
from madgwick_py.madgwickahrs import MadgwickAHRS  # pylint: disable=relative-import

UNKNOWN_VARIANCE = 0

# If accuracy is updated these need updating from the datasheet
LINEAR_ACCELERATION_COVARIANCE = [0.061, 0, 0, 0, 0.061, 0, 0, 0, 0.061]
ANGULAR_VELOCITY_COVARIANCE = [0.00875, 0, 0, 0, 0.00875, 0, 0, 0, 0.00875]
# This is an estimate of the average covariance, assuming a normal distribution.
ORIENTATION_COVARIANCE = [0.139, 0, 0, 0, 0.139, 0, 0, 0, 0.139]

# Rough estimate based on measurements with Pi-pucks, calibration script should be run to get
# accurate results.
CALIBRATION_DATA_DEFAULT = {
    "magnetometer": {
        "y": -2.981,
        "x": 0.187,
        "z": -1.628
    },
    "gyro": {
        "y": 0.0211,
        "x": -0.0101,
        "z": 0.00372
    },
    "accelerometer": {
        "y": -0.441,
        "x": -0.391,
        "z": -0.200
    }
}

REFERENCE_FRAME_ID = "imu_sensor"

# Increasing beta will increase confidence in IMU values, this will produce faster results but
# they may be less accurate. Reducing it will reduce confidence and will slow response time, but
# increase accuracy.
AHRS_BETA = 4


class PiPuckImuServer(object):  # pylint: disable=too-many-instance-attributes
    """ROS Node to publish data from the Pi-puck's IMU."""

    def __init__(self):
        """Initialise IMU server node."""
        rospy.on_shutdown(self.close_sensor)

        rospy.init_node("imu")

        # Switch to determine if full orientation is calculated or just rotation around z axis.
        self._calculate_full_orientation = bool(rospy.get_param("~calculate_full_orientation",
                                                                True))

        # Unimplemented switch to control removal of gravity.
        self._remove_gravity = bool(rospy.get_param("~remove_gravity", False))

        # Handle transformation tree prefix.
        tf_prefix_key = rospy.search_param("tf_prefix")
        if tf_prefix_key:
            self._tf_prefix = rospy.get_param(tf_prefix_key, None)
        else:
            self._tf_prefix = None
        self._tf_prefix = rospy.get_param(tf_prefix_key, None)

        if self._tf_prefix is not None and not self._tf_prefix.endswith("/"):
            self._tf_prefix += "/"

        # Rate of sensor data transmission
        self._rate = rospy.Rate(rospy.get_param('~rate', 10))

        # Rate of collection of sample data
        self._raw_sample_rate = int(rospy.get_param('~sample_rate', 128))
        self._sample_rate = rospy.Rate(self._raw_sample_rate)

        self._sensor_imu_publisher = rospy.Publisher('imu/imu', Imu, queue_size=10)
        self._sensor_temperature_publisher = rospy.Publisher('imu/temperature',
                                                             Temperature,
                                                             queue_size=10)
        self._sensor_magnetic_publisher = rospy.Publisher('imu/magnetic',
                                                          MagneticField,
                                                          queue_size=10)

        self._sensor = None

        # Handle IMU calibration
        calibration_file = None

        if path.isfile("calibration.json"):
            calibration_file = "calibration.json"
        elif path.isfile(path.join(path.dirname(__file__), "calibration.json")):
            calibration_file = path.join(path.dirname(__file__), "calibration.json")

        if calibration_file:
            with open(calibration_file, "rb") as calibration_file_handle:
                self._calibration = load(calibration_file_handle)
        else:
            self._calibration = CALIBRATION_DATA_DEFAULT

        # Setup data storage variables.
        if self._calculate_full_orientation:
            self._orientation_filter = MadgwickAHRS(sampleperiod=1.0 / float(self._raw_sample_rate),
                                                    beta=AHRS_BETA)
        self._acceleration_result = (0, 0, 0)
        self._orientation_quaternion = Quaternion(w=1, x=0, y=0, z=0)
        self._gyro_result = (0, 0, 0)
        self._magnetometer_result = (0, 0, 0)

    def close_sensor(self):
        """Close the sensor after the ROS Node is shutdown."""
        self._sensor.close()

    def open_sensor(self):
        """Open the sensor."""
        self._sensor = LSM9DS1()

    @staticmethod
    def euler_to_quaternion(yaw, pitch, roll):
        """Convert euler angles of pitch, roll, and yaw to a quaternion.

        Based on code from
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.

        We don't use tf.transforms here to reduce dependence on tf and increase performance on this
        simple task.
        """
        c_y = math.cos(yaw * 0.5)
        s_y = math.sin(yaw * 0.5)
        c_p = math.cos(pitch * 0.5)
        s_p = math.sin(pitch * 0.5)
        c_r = math.cos(roll * 0.5)
        s_r = math.sin(roll * 0.5)

        q_w = c_y * c_p * c_r + s_y * s_p * s_r
        q_z = c_y * c_p * s_r - s_y * s_p * c_r
        q_y = s_y * c_p * s_r + c_y * s_p * c_r
        q_x = s_y * c_p * c_r - c_y * s_p * s_r

        return Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)

    @staticmethod
    def calculate_heading(magnetometer_reading):
        """Calculate the current orientation using magnetometer.

        Currently only supports yaw calculation e.g. rotation about the z axis.
        """
        mag_x, mag_y, _ = magnetometer_reading

        if mag_y == 0:
            if mag_x > 0:
                x_y_direction = math.radians(180)
            else:  # x <= 0
                x_y_direction = math.radians(0)
        elif mag_y < 0:
            x_y_direction = math.radians(90) - math.atan(mag_x / mag_y)
        else:  # y > 0
            x_y_direction = math.radians(270) - math.atan(mag_x / mag_y)

        return x_y_direction

    @staticmethod
    def calculate_heading_quaternion(magnetometer_reading):
        """Calculate the current orientation as a quaternion using magnetometer.

        Currently only supports yaw calculation e.g. rotation about the z axis.
        """
        return PiPuckImuServer.euler_to_quaternion(
            yaw=PiPuckImuServer.calculate_heading(magnetometer_reading), pitch=0, roll=0)

    def _sample_thread(self):
        """Run sampling thread, to be run in a separate thread to handle sampling of sensors."""
        while not rospy.is_shutdown():
            # Acceleration is already in m/s^2 so no conversion needed,
            # just unpacking
            acceleration_result = self._sensor.acceleration
            acceleration_x, acceleration_y, acceleration_z = acceleration_result
            acceleration_x, acceleration_y, acceleration_z = acceleration_x - self._calibration[
                "accelerometer"]["x"], acceleration_y - self._calibration["accelerometer"][
                    "y"], acceleration_z - self._calibration["accelerometer"]["z"]

            # Gyro is in degrees/second so conversion to rads/sec is needed,
            # as well as unpacking
            gyro_result = [deg * (math.pi / 180.0) for deg in self._sensor.gyro]
            gyro_x, gyro_y, gyro_z = gyro_result
            gyro_x, gyro_y, gyro_z = gyro_x - self._calibration["gyro"][
                "x"], gyro_y - self._calibration["gyro"]["y"], gyro_z - self._calibration["gyro"][
                    "z"]

            # Magnetometer values can be used to calculate our orientation rotation about the z
            # axis (yaw).
            # Futher work is needed to calculate pitch and roll from other available sensors.
            magnetometer_result = self._sensor.magnetic
            # magnetometer_quaternion = self.calculate_heading_quaternion(magnetometer_result)

            # Apply calibration to magnetometer result
            magnetometer_result = (magnetometer_result[0] - self._calibration["magnetometer"]["x"],
                                   magnetometer_result[1] - self._calibration["magnetometer"]["y"],
                                   magnetometer_result[2] - self._calibration["magnetometer"]["z"])

            if self._calculate_full_orientation:
                previous_orientation_quaternion = self._orientation_filter.quaternion
                self._orientation_filter.update(gyro_result, acceleration_result,
                                                magnetometer_result)

                if any(map(math.isnan, self._orientation_filter.quaternion)):
                    self._orientation_filter = MadgwickAHRS(
                        sampleperiod=1.0 / float(self._raw_sample_rate),
                        quaternion=previous_orientation_quaternion,
                        beta=AHRS_BETA)

                filter_w, filter_x, filter_y, filter_z = self._orientation_filter.quaternion
                self._orientation_quaternion = Quaternion(w=filter_w,
                                                          x=filter_x,
                                                          y=filter_y,
                                                          z=filter_z)
            else:
                self._orientation_quaternion = PiPuckImuServer.calculate_heading_quaternion(
                    magnetometer_result)

            self._gyro_result = (gyro_x, gyro_y, gyro_z)

            if self._remove_gravity:
                pass  # Not yet implemented

            self._acceleration_result = (acceleration_x, acceleration_y, acceleration_z)

            self._magnetometer_result = magnetometer_result

            self._sample_rate.sleep()

    def run(self):
        """Run the sensor server."""
        self.open_sensor()

        # We need a separate sensor reading thread as the sample rate must be relatively high
        # (> 64 hrz), ideally close to 256 hrz. If we sent IMU data at this rate the overheads of
        # sending would slow the sensor reading rate, hence we use a separate thread.
        sensor_sample_thread = Thread(target=self._sample_thread)
        sensor_sample_thread.start()

        while not rospy.is_shutdown():
            if self._tf_prefix:
                tf_reference_frame = self._tf_prefix + REFERENCE_FRAME_ID
            else:
                tf_reference_frame = REFERENCE_FRAME_ID

            # Temperature is in degrees C so no conversion needed
            temperature_message = Temperature(temperature=self._sensor.temperature,
                                              variance=UNKNOWN_VARIANCE)
            temperature_message.header.frame_id = tf_reference_frame
            self._sensor_temperature_publisher.publish(temperature_message)

            acceleration_x, acceleration_y, acceleration_z = self._acceleration_result
            gyro_x, gyro_y, gyro_z = self._gyro_result
            magnetometer_x, magnetometer_y, magnetometer_z = self._magnetometer_result
            orientation_quaternion = self._orientation_quaternion

            imu_message = Imu(linear_acceleration_covariance=LINEAR_ACCELERATION_COVARIANCE,
                              linear_acceleration=Vector3(x=acceleration_x,
                                                          y=acceleration_y,
                                                          z=acceleration_z),
                              angular_velocity_covariance=ANGULAR_VELOCITY_COVARIANCE,
                              angular_velocity=Vector3(x=gyro_x, y=gyro_y, z=gyro_z),
                              orientation=orientation_quaternion,
                              orientation_covariance=ORIENTATION_COVARIANCE)
            imu_message.header.frame_id = tf_reference_frame

            self._sensor_imu_publisher.publish(imu_message)

            magnetic_message = MagneticField(
                magnetic_field=Vector3(x=magnetometer_x, y=magnetometer_y, z=magnetometer_z))
            magnetic_message.header.frame_id = tf_reference_frame

            self._sensor_magnetic_publisher.publish(magnetic_message)

            self._rate.sleep()

        sensor_sample_thread.join()


if __name__ == "__main__":
    PiPuckImuServer().run()
