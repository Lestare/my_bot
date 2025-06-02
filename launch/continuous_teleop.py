#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time

class TankTeleop:
    def __init__(self):
        self.pub = rclpy.Publisher('cmd_vel', Twist, queue_size=1)
        self.linear_speed = rclpy.get_param("~linear", 0.5)  # м/с
        self.angular_speed = rclpy.get_param("~angular", 1.0)  # рад/с
        self.settings = termios.tcgetattr(sys.stdin)
        
        rclpy.loginfo(f"Tank-style teleop initialized")
        rclpy.loginfo(f"Linear speed: {self.linear_speed} m/s")
        rclpy.loginfo(f"Angular speed: {self.angular_speed} rad/s")
        rclpy.loginfo("Controls:")
        rclpy.loginfo("  W: Move forward")
        rclpy.loginfo("  S: Move backward")
        rclpy.loginfo("  A: Rotate counter-clockwise (tank turn)")
        rclpy.loginfo("  D: Rotate clockwise (tank turn)")
        rclpy.loginfo("  Q: Increase linear speed")
        rclpy.loginfo("  Z: Decrease linear speed")
        rclpy.loginfo("  E: Increase angular speed")
        rclpy.loginfo("  C: Decrease angular speed")
        rclpy.loginfo("  SPACE: Emergency stop")
        rclpy.loginfo("  CTRL+C: Exit")

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if sys.stdin in select.select([sys.stdin], [], [], 0)[0] else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while not rclpy.is_shutdown():
                key = self.getKey()
                twist = Twist()
                
                # Движение вперед/назад
                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                
                # Танковые повороты на месте
                elif key == 'a':
                    twist.angular.z = self.angular_speed  # Поворот против часовой
                elif key == 'd':
                    twist.angular.z = -self.angular_speed  # Поворот по часовой
                
                # Настройка скоростей
                elif key == 'q':
                    self.linear_speed = min(2.0, self.linear_speed + 0.1)
                    rclpy.loginfo(f"Linear speed: {self.linear_speed:.1f} m/s")
                elif key == 'z':
                    self.linear_speed = max(0.1, self.linear_speed - 0.1)
                    rclpy.loginfo(f"Linear speed: {self.linear_speed:.1f} m/s")
                elif key == 'e':
                    self.angular_speed = min(3.0, self.angular_speed + 0.2)
                    rclpy.loginfo(f"Angular speed: {self.angular_speed:.1f} rad/s")
                elif key == 'c':
                    self.angular_speed = max(0.2, self.angular_speed - 0.2)
                    rclpy.loginfo(f"Angular speed: {self.angular_speed:.1f} rad/s")
                
                # Экстренная остановка
                elif key == ' ':
                    twist.linear.x = 0
                    twist.angular.z = 0
                elif key == '\x03':  # CTRL+C
                    break
                
                self.pub.publish(twist)
                
        finally:
            # Остановка при выходе
            twist = Twist()
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__":
    rclpy.init_node("tank_teleop")
    teleop = TankTeleop()
    teleop.run()