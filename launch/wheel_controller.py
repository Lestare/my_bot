#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'wheel_velocities', 10)
        self.get_logger().info("Контроллер колес запущен")

    def cmd_callback(self, msg):
        # Конвертация в скорости колес (4WD кинематика)
        out_msg = Twist()
        
        # Линейная и угловая скорости
        vx = msg.linear.x
        wz = msg.angular.z
        
        # Рассчет скоростей колес
        L = 0.76  # Ширина колеи
        R = 0.17  # Радиус колеса
        
        out_msg.linear.x = (vx - wz * L / 2) / R  # Левые колеса
        out_msg.linear.y = (vx + wz * L / 2) / R  # Правые колеса
        
        self.vel_pub.publish(out_msg)

def main():
    rclpy.init()
    node = WheelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()