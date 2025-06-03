#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.5  # м/с
        self.angular_speed = 0.8  # рад/с

        self.get_logger().info("Keyboard control initialized. Use WASD to move, Q to quit.")

    def run(self):
        try:
            while rclpy.ok():
                key = input("Command [WASD]: ").lower()
                twist = Twist()

                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                elif key == 'q':
                    self.get_logger().info("User exit")
                    break
                else:
                    continue

                self.cmd_vel_pub.publish(twist)
        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt received, shutting down.")

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
