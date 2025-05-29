#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        
        # Подписка на команды движения с сервера
        self.cmd_sub = self.create_subscription(
            Twist,
            'remote_cmd_vel',
            self.cmd_callback,
            10
        )
        
        # Публикация одометрии на сервер
        self.odom_pub = self.create_publisher(Odometry, 'remote_odom', 10)
        
        # Подписка на локальную одометрию
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info("Rover Controller: Ready to receive commands")

    def cmd_callback(self, msg):
        # Перенаправляем команды в локальный топик управления
        cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        cmd_pub.publish(msg)
        self.get_logger().info(f"Command received: Linear={msg.linear.x}, Angular={msg.angular.z}")

    def odom_callback(self, msg):
        # Отправляем одометрию на сервер
        self.odom_pub.publish(msg)

def main():
    rclpy.init()
    node = RoverController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()