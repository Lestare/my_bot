#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Remote(Node):
    def __init__(self):
        super().__init__('remote')
        
        # Кнопки управления
        self.cmd_pub = self.create_publisher(Twist, 'remote_cmd', 10)
        
        # Экран с положением ровера
        self.create_subscription(Odometry, 'remote_odom', self.odom_callback, 10)
        
        print("Пульт готов! Управляю ровером...")
        self.timer = self.create_timer(1.0, self.send_command)  # Шлем команды каждую секунду

    def send_command(self):
        cmd = Twist()
        cmd.linear.x = 0.5   # Всегда едем вперед
        cmd.angular.z = 0.3  # Слегка поворачиваем
        self.cmd_pub.publish(cmd)
        print("Отправил команду: ВПЕРЕД И ПОВОРОТ")

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
