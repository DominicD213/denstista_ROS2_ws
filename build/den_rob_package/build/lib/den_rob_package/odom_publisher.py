#!/usr/bin/env python3

import rclpy
from math import sin, cos
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish at 10 Hz
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_odometry)

        # Simulated state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.1       # forward velocity
        self.vth = 0.1      # rotational velocity

        self.last_time = self.get_clock().now()

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Update simulated position
        delta_x = self.vx * dt * cos(self.th)
        delta_y = self.vx * dt * sin(self.th)
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Quaternion from yaw
        odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.th)

        # Broadcast the transform
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation.x = odom_quat[0]
        odom_trans.transform.rotation.y = odom_quat[1]
        odom_trans.transform.rotation.z = odom_quat[2]
        odom_trans.transform.rotation.w = odom_quat[3]

        self.tf_broadcaster.sendTransform(odom_trans)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'

        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = Quaternion(
            x=odom_quat[0],
            y=odom_quat[1],
            z=odom_quat[2],
            w=odom_quat[3]
        )

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear = Vector3(x=self.vx, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.vth)

        self.publisher_.publish(odom)
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
