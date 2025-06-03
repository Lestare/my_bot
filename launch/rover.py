#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.create_timer(0.05, self.update_odometry)
        self.get_logger().info("Rover emulator ready")

    def cmd_callback(self, msg):
        self.last_cmd = msg

    def update_odometry(self):
        if hasattr(self, 'last_cmd'):
            dt = 0.05
            self.theta += self.last_cmd.angular.z * dt
            self.x += self.last_cmd.linear.x * math.cos(self.theta) * dt
            self.y += self.last_cmd.linear.x * math.sin(self.theta) * dt

        # TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta/2)
        t.transform.rotation.w = math.cos(self.theta/2)
        self.tf_broadcaster.sendTransform(t)
        
        # Odometry
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = Rover()
    rclpy.spin(node)
    rclpy.shutdown()