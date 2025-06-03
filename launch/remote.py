#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Operator(Node):
    def __init__(self):
        super().__init__('operator')
        
        # Публикатор команд для ровера
        self.cmd_pub = self.create_publisher(Twist, 'remote_cmd', 10)
        
        # Подписка на команды от teleop
        self.create_subscription(Twist, 'cmd_vel', self.teleop_callback, 10)
        
        self.get_logger().info("Оператор готов к управлению!")

    def teleop_callback(self, msg):
        # Пересылаем команды напрямую на ровер
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Отправляю команду: X={msg.linear.x}, Z={msg.angular.z}")

def main():
    rclpy.init()
    node = Operator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()