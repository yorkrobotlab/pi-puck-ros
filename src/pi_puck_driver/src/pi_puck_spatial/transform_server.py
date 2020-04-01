#!/usr/bin/env python
"""ROS Node to publish transforms based on data from sensors."""

from collections import namedtuple
from xml.etree import ElementTree as ETREE
from math import cos, sin, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Int64
from geometry_msgs.msg import Vector3

from imu_server import PiPuckImuServer  # pylint: disable=relative-import

calculate_heading = PiPuckImuServer.calculate_heading  # pylint: disable=invalid-name

StaticTransform = namedtuple("StaticTransform", ["parent_link", "child_link", "xyz", "rpy"])

WHEEL_DIAMETER = 4.0  # cm
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * pi / 100.0  # in metres
WHEEL_DISTANCE = 0.053  # the distance between the wheels in metres
MOTOR_STEP_DISTANCE = WHEEL_CIRCUMFERENCE / 1000.0  # 1 turn should be 1000 steps


class PiPuckTransformServer(object):  # pylint: disable=too-many-instance-attributes
    """ROS Node to publish data from the Pi-puck's IMU."""

    def __init__(self):
        """Initialise TF server node."""
        rospy.init_node("tf_broadcaster")

        self._rate = rospy.Rate(rospy.get_param('~rate', 10))
        self._use_imu = bool(rospy.get_param('~use_imu', True))
        self._use_hybrid_position = bool(rospy.get_param('~use_hybrid_position', True))
        self._publish_hybrid_position = bool(rospy.get_param('~publish_hybrid_position', True))
        self._use_map_as_parent = bool(rospy.get_param('~use_map_as_parent', False))

        self._robot_description_raw = rospy.get_param("robot_description", None)
        self._tf_prefix = rospy.get_param("tf_prefix", None)

        if self._tf_prefix is not None and not self._tf_prefix.endswith("/"):
            self._tf_prefix += "/"

        if self._publish_hybrid_position:
            self._odometry_pub = rospy.Publisher("tf_broadcaster/odometry", Odometry, queue_size=10)

        self._publish_static_transforms = rospy.get_param("~publish_static_transforms", True)
        if not isinstance(self._publish_static_transforms, bool):
            self._publish_static_transforms = bool(self._publish_static_transforms)

        if self._robot_description_raw:
            self._robot_description = ETREE.fromstring(self._robot_description_raw)
        else:
            self._robot_description = None

        if self._publish_static_transforms:
            self._static_transforms = []

            self._find_static_transforms()

        self._broadcaster = tf.TransformBroadcaster()

        self._current_xyz = (0, 0, 0)
        self._current_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self._current_angular = Vector3(0, 0, 0)
        self._current_linear = Vector3(0, 0, 0)
        self._steps_right = None
        self._steps_left = None
        self._previous_steps_right = None
        self._previous_steps_left = None
        self._magnetic_orientation = None
        self._previous_theta = None

    def _find_static_transforms(self):

        def _is_fixed(joint_to_check):
            return joint_to_check.get("type").lower() == "fixed"

        for joint in filter(_is_fixed, self._robot_description.findall("joint")):
            parent = joint.find("parent")
            child = joint.find("child")
            origin = joint.find("origin")
            if parent is not None and child is not None and origin is not None:
                parent_link = parent.get("link")
                child_link = child.get("link")
                origin_xyz = origin.get("xyz")
                origin_rpy = origin.get("rpy")
                if parent_link is not None and child_link is not None and origin_xyz is not None:
                    origin_xyz = tuple(map(float, origin_xyz.split(" ")))
                    if origin_rpy is None:
                        origin_rpy = (0, 0, 0)
                    else:
                        origin_rpy = tuple(map(float, origin_rpy.split(" ")))
                    self._static_transforms.append(
                        StaticTransform(parent_link=parent_link,
                                        child_link=child_link,
                                        xyz=origin_xyz,
                                        rpy=origin_rpy))

    def send_static_transforms(self):
        """Send static transforms for fixed joints."""
        for static_transform in self._static_transforms:
            self._broadcaster.sendTransform(
                translation=static_transform.xyz,
                rotation=tf.transformations.quaternion_from_euler(*static_transform.rpy),
                time=rospy.Time.now(),
                child=self._tf_prefix + static_transform.child_link,
                parent=self._tf_prefix + static_transform.parent_link)

    def send_pi_puck_transform(self):
        """Send transform for Pi-puck."""
        self._broadcaster.sendTransform(
            translation=self._current_xyz,
            rotation=self._current_quaternion,
            time=rospy.Time.now(),
            child=self._tf_prefix + "base_link",
            parent="map" if self._use_map_as_parent else self._tf_prefix + "odom")

    def imu_data_callback(self, data):
        """IMU data callback handler."""
        self._current_quaternion = (data.orientation.w, data.orientation.x, data.orientation.y,
                                    data.orientation.z)
        self._current_angular = data.angular_velocity
        self._current_linear = data.linear_acceleration

    def motor_odometry_data_callback(self, data):
        """Motor data callback handler."""
        if not self._use_hybrid_position:
            self._current_xyz = (data.pose.pose.position.x, data.pose.pose.position.y,
                                 data.pose.pose.position.z)
        if not self._use_imu:
            self._current_quaternion = (data.pose.pose.orientation.w, data.pose.pose.orientation.x,
                                        data.pose.pose.orientation.y, data.pose.pose.orientation.z)

    def magnetometer_callback(self, data):
        """Magnetometer data callback."""
        raw_heading = calculate_heading(
            (data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z))
        self._magnetic_orientation = (2 * pi) - ((raw_heading + pi) % (2 * pi))

    def motor_steps_left_data_callback(self, data):
        """Motor data callback handler."""
        self._steps_left = data.data

    def motor_steps_right_data_callback(self, data):
        """Motor data callback handler."""
        self._steps_right = data.data

    def calculate_hybrid_position(self):
        """Calculate position using both Odometry and the IMU."""
        steps_left = self._steps_left
        steps_right = self._steps_right
        previous_steps_left = self._previous_steps_left
        previous_steps_right = self._previous_steps_right
        previous_theta = self._previous_theta
        current_theta = self._magnetic_orientation

        has_magnetic_data = current_theta is not None
        has_theta = previous_theta is not None
        has_steps = previous_steps_left is not None and previous_steps_right is not None

        if has_steps and has_theta and has_magnetic_data:
            delta_left = (steps_left - previous_steps_left) * MOTOR_STEP_DISTANCE
            delta_right = (steps_right - previous_steps_right) * MOTOR_STEP_DISTANCE

            delta_theta = current_theta - previous_theta
            delta_steps = (delta_right + delta_left) / 2.0

            delta_estimate_x = delta_steps * cos(previous_theta + delta_theta / 2.0)
            delta_estimate_y = delta_steps * sin(previous_theta + delta_theta / 2.0)

            c_x, c_y, c_z = self._current_xyz

            self._current_xyz = (c_x + delta_estimate_x, c_y + delta_estimate_y, c_z)

        self._previous_steps_left = steps_left
        self._previous_steps_right = steps_right
        self._previous_theta = current_theta

    def publish_hybrid_position(self):
        """Publish the estimated hybrid position as odometry."""
        odometry_message = Odometry()
        odometry_message.header.frame_id = self._tf_prefix + "base_link"

        (odometry_message.pose.pose.position.x, odometry_message.pose.pose.position.y,
         odometry_message.pose.pose.position.z) = self._current_xyz

        (odometry_message.pose.pose.orientation.w, odometry_message.pose.pose.orientation.x,
         odometry_message.pose.pose.orientation.y,
         odometry_message.pose.pose.orientation.z) = self._current_quaternion

        odometry_message.twist.twist.angular = self._current_angular
        odometry_message.twist.twist.angular = self._current_linear

        self._odometry_pub.publish(odometry_message)

    def run(self):
        """Run the transform server."""
        if self._use_imu:
            rospy.Subscriber("navigation/spatial/imu/imu", Imu, self.imu_data_callback)
        if not self._use_hybrid_position or not self._use_imu:
            rospy.Subscriber("navigation/motors/odometry", Odometry,
                             self.motor_odometry_data_callback)
        if self._use_hybrid_position:
            rospy.Subscriber("navigation/motors/real_steps_left", Int64,
                             self.motor_steps_left_data_callback)
            rospy.Subscriber("navigation/motors/real_steps_right", Int64,
                             self.motor_steps_right_data_callback)
            rospy.Subscriber("navigation/spatial/imu/magnetic", MagneticField,
                             self.magnetometer_callback)

        while not rospy.is_shutdown():
            if self._publish_static_transforms:
                self.send_static_transforms()
            if self._use_hybrid_position:
                self.calculate_hybrid_position()
            if self._publish_hybrid_position:
                self.publish_hybrid_position()
            self.send_pi_puck_transform()
            self._rate.sleep()


if __name__ == '__main__':
    PiPuckTransformServer().run()
