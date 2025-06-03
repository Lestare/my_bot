#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import sys, select, tty, termios

class Teleop:
    def __init__(self):
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.linear_speed = 0.5   # м/с
        self.angular_speed = 0.8  # рад/с
        self.settings = termios.tcgetattr(sys.stdin)
        
    def send_command(self):
        self.cmd_vel_pub.publish(self.twist)
        
    def run(self):
        try:
            tty.setraw(sys.stdin.fileno())
            print("Управление (зажимайте клавиши):")
            print("  W - вперед")
            print("  S - назад")
            print("  A - влево")
            print("  D - вправо")
            print("  Space - стоп")
            print("  Ctrl+C - выход")
            
            while not rclpy.is_shutdown():
                key = sys.stdin.read(1)
                
                # Обработка зажатия клавиш
                if key == 'w':
                    self.twist.linear.x = self.linear_speed
                elif key == 's':
                    self.twist.linear.x = -self.linear_speed
                elif key == 'a':
                    self.twist.angular.z = self.angular_speed
                elif key == 'd':
                    self.twist.angular.z = -self.angular_speed
                elif key == ' ':
                    self.twist = Twist()
                
                self.send_command()
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    rclpy.init()
    teleop = Teleop()
    teleop.run()