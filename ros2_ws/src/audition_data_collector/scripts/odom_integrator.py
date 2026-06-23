#!/usr/bin/env python3


import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry


class OdomIntegrator(Node):

    def __init__(self):
        super().__init__('odom_integrator')

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = self.get_parameter('start_yaw').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.last_time = time.time()

        self.twist_sub = self.create_subscription(
            Twist, '/rover_twist_odo', self.twist_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Republish at a steady rate even if velocity hasn't changed,
        # so controller.cpp always has fresh /odom data.
        self.timer = self.create_timer(0.05, self.publish_odom)  # 20 Hz

        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_angular_z = 0.0

        self.get_logger().info(
            f'Odom integrator started at ({self.x}, {self.y}, yaw={self.yaw})')

    def twist_callback(self, msg: Twist):
        self.last_linear_x = msg.linear.x
        self.last_linear_y = msg.linear.y
        self.last_angular_z = msg.angular.z

    def publish_odom(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Integrate velocity -> position (simple Euler integration)
        delta_x = (self.last_linear_x * math.cos(self.yaw) -
                   self.last_linear_y * math.sin(self.yaw)) * dt
        delta_y = (self.last_linear_x * math.sin(self.yaw) +
                   self.last_linear_y * math.cos(self.yaw)) * dt
        delta_yaw = self.last_angular_z * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2.0 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2.0 * math.pi

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        odom.twist.twist.linear.x = self.last_linear_x
        odom.twist.twist.linear.y = self.last_linear_y
        odom.twist.twist.angular.z = self.last_angular_z

        self.odom_pub.publish(odom)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = OdomIntegrator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
