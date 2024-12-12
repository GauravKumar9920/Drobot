#!/usr/bin/env python3

from time import time
import rclpy

import tf2_ros

from rclpy import duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class TransformPublisher(Node):
    def __init__(self):
        super().__init__("tf_publisher")

        qos_odom = DurabilityPolicy.SYSTEM_DEFAULT

        qos_pose = DurabilityPolicy.SYSTEM_DEFAULT

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry", self.odom_cb, qos_odom
        )
        # self.movus_pose_sub = self.create_subscription(TransformStamped, "/model/movus/pose", self.movus_pose_cb, qos_pose)

        self.tf_br = tf2_ros.TransformBroadcaster(self)

    def odom_cb(self, odom_msg):
        tf_s = TransformStamped()
        tf_s.header.stamp = (self.get_clock().now() + Duration(seconds=0.5)).to_msg()
        tf_s.header.frame_id = "odom"
        tf_s.child_frame_id = "base_link"

        tf_s.transform.translation.x = odom_msg.pose.pose.position.x
        tf_s.transform.translation.y = odom_msg.pose.pose.position.y
        tf_s.transform.translation.z = 0.0

        tf_s.transform.rotation.x = odom_msg.pose.pose.orientation.x
        tf_s.transform.rotation.y = odom_msg.pose.pose.orientation.y
        tf_s.transform.rotation.z = odom_msg.pose.pose.orientation.z

        self.tf_br.sendTransform(tf_s)

    def movus_pose_cb(self, pose_msg):
        transform_stamped = TransformStamped()
        base_link_to_movus_tf = TransformStamped()

        transform_stamped.header.frame_id = pose_msg.header.frame_id
        transform_stamped.child_frame_id = pose_msg.child_frame_id

        transform_stamped.transform = pose_msg.transform

        if transform_stamped.header.frame_id == "station":
            pass
        else:
            transform_stamped.header.stamp = self.get_clock().now().to_msg()

            base_link_to_movus_tf.header.stamp = self.get_clock().now().to_msg()
            base_link_to_movus_tf.header.frame_id = "base_link"
            base_link_to_movus_tf.child_frame_id = "movus"

            base_link_to_movus_tf.transform.translation.x = 0.0
            base_link_to_movus_tf.transform.translation.y = 0.0
            base_link_to_movus_tf.transform.translation.z = 0.0

            base_link_to_movus_tf.transform.rotation.x = 0.0
            base_link_to_movus_tf.transform.rotation.y = 0.0
            base_link_to_movus_tf.transform.rotation.z = 0.0

            self.tf_br.sendTransform([transform_stamped, base_link_to_movus_tf])


if __name__ == "__main__":
    rclpy.init()

    tf_pub = TransformPublisher()

    try:
        rclpy.spin(tf_pub)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
