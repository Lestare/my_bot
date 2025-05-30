#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        
        # Подписка на команды с оператора
        self.create_subscription(Twist, 'remote_cmd', self.cmd_callback, 10)
        
        # Публикация фиктивной одометрии (для теста)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Имитация положения
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Таймер для публикации одометрии
        self.create_timer(0.1, self.publish_odometry)
        
        self.get_logger().info("Ровер готов к работе!")

    def cmd_callback(self, msg):
        # Здесь реальное управление моторами
        # Для теста - просто логируем команду
        self.get_logger().info(f"Принял команду: X={msg.linear.x}, Z={msg.angular.z}")
        
        # Обновляем "позицию" для визуализации
        self.theta += msg.angular.z * 0.1
        self.x += msg.linear.x * math.cos(self.theta) * 0.1
        self.y += msg.linear.x * math.sin(self.theta) * 0.1

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        
        # Преобразуем угол в кватернион
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        
        self.odom_pub.publish(msg)

def main():
    rclpy.init()
    node = Rover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()