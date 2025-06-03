#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios

class Teleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.linear_speed = 0.5   # м/с
        self.angular_speed = 0.8  # рад/с
        self.settings = termios.tcgetattr(sys.stdin)
        
    def send_command(self):
        self.publisher_.publish(self.twist)
        
    def run(self):
        try:
            tty.setraw(sys.stdin.fileno())
            print("Управление (зажимайте клавиши):")
            print("  W - вперед")
            print("  S - назад")
            print("  A - влево")
            print("  D - вправо")
            print("  Space - стоп")
            print("  Q - выход")
            
            while rclpy.ok():
                # Проверяем доступность ввода
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == 'q':  # Выход по 'q'
                        break
                        
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
            # Останавливаем ровер перед выходом
            self.twist = Twist()
            self.send_command()

def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()