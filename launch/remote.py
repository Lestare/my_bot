#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Remote(Node):
    def __init__(self):
        super().__init__('remote')
        
        # Публикатор команд для ровера
        self.cmd_pub = self.create_publisher(Twist, 'remote_cmd', 10)
        
        # Подписка на одометрию от ровера
        self.create_subscription(Odometry, 'remote_odom', self.odom_callback, 10)
        
        # Подписка на команды от teleop
        self.create_subscription(Twist, 'cmd_vel', self.teleop_callback, 10)
        
        print("Пульт готов! Запусти teleop в другом терминале")
        
    def teleop_callback(self, msg):
        """Получаем команды от teleop и пересылаем на ровер"""
        self.cmd_pub.publish(msg)
        print(f"Отправляю команду: X={msg.linear.x}, Z={msg.angular.z}")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        print(f"Ровер здесь: X={pos.x:.1f}, Y={pos.y:.1f}")

def main():
    rclpy.init()
    node = Remote()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()