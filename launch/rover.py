#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        
        # Подписка на команды
        self.create_subscription(Twist, 'remote_cmd', self.cmd_callback, 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish_transforms)   
        
        self.get_logger().info("Ровер готов к работе!")

    def cmd_callback(self, msg):
        # Обновляем положение на основе команд
        self.theta += msg.angular.z * 0.05
        self.x += msg.linear.x * math.cos(self.theta) * 0.05
        self.y += msg.linear.x * math.sin(self.theta) * 0.05
        self.get_logger().info(f"Позиция: X={self.x:.2f}, Y={self.y:.2f}")

    def publish_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.z = 0.0
        # Ориентация остается неизменной
        self.tf_broadcaster.sendTransform(t)
 
        

def main():
    rclpy.init()
    node = Rover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()