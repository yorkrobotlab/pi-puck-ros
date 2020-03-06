#!/usr/bin/env python
"""ROS Node to publish transforms based on data from sensors."""

import rospy
import tf

from sensor_msgs.msg import Imu

from collections import namedtuple
from xml.etree import ElementTree as ETREE

StaticTransform = namedtuple("StaticTransform", ["parent_link", "child_link", "xyz", "rpy"])


class PiPuckTransformServer(object):
    """ROS Node to publish data from the Pi-puck's IMU."""

    def __init__(self):
        """Initialise TF server node."""
        rospy.init_node("tf_broadcaster")

        self._rate = rospy.Rate(rospy.get_param('~rate', 5))

        self._robot_description_raw = rospy.get_param("robot_description", None)
        self._tf_prefix = rospy.get_param("tf_prefix", None)

        if self._tf_prefix is not None and not self._tf_prefix.endswith("/"):
            self._tf_prefix += "/"

        self._publish_static_transforms = rospy.get_param("publish_static_transforms", True)
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
                        StaticTransform(parent_link=parent_link, child_link=child_link, xyz=origin_xyz, rpy=origin_rpy))

    def send_static_transforms(self):
        """Send static transforms for fixed joints."""
        for static_transform in self._static_transforms:
            self._broadcaster.sendTransform(translation=static_transform.xyz,
                                            rotation=tf.transformations.quaternion_from_euler(*static_transform.rpy),
                                            time=rospy.Time.now(),
                                            child=self._tf_prefix + static_transform.child_link,
                                            parent=self._tf_prefix + static_transform.parent_link)

    def send_pi_puck_transform(self):
        """Send transform for Pi-puck."""
        self._broadcaster.sendTransform(translation=self._current_xyz,
                                        rotation=self._current_quaternion,
                                        time=rospy.Time.now(),
                                        child=self._tf_prefix + "base_link",
                                        parent="map")

    def imu_data_callback(self, data):
        """IMU data callback handler."""
        self._current_quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        self.send_pi_puck_transform()

    def run(self):
        """Run the transform server."""
        rospy.Subscriber("navigation/spatial/imu/imu", Imu, self.imu_data_callback)
        while not rospy.is_shutdown():
            if self._publish_static_transforms:
                self.send_static_transforms()
            self.send_pi_puck_transform()
            self._rate.sleep()


if __name__ == '__main__':
    tf_server = PiPuckTransformServer()
    tf_server.run()
