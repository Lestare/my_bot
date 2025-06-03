#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster

class RoverEmulator(Node):
    def __init__(self):
        super().__init__('rover_emulator')
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_cmd_time = self.get_clock().now()
        
        self.create_timer(0.05, self.update_odometry)
        self.get_logger().info("Rover emulator started")

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def update_odometry(self):
        # Если не было команд более 0.5 секунд, останавливаемся
        if not hasattr(self, 'last_cmd') or (self.get_clock().now() - self.last_cmd_time).nanoseconds > 0.5e9:
            return

        dt = 0.05  # 20 Гц
        self.theta += self.last_cmd.angular.z * dt
        self.x += self.last_cmd.linear.x * math.cos(self.theta) * dt
        self.y += self.last_cmd.linear.x * math.sin(self.theta) * dt

        # Публикация TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.tf_broadcaster.sendTransform(t)
        
        # Публикация Odometry
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = RoverEmulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()