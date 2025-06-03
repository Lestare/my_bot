#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RelayCmd(Node):
    def __init__(self):
        super().__init__('relay_cmd')
        self.sub = self.create_subscription(Twist, 'remote_cmd', self.callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Relay ready: remote_cmd -> cmd_vel")

    def callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = RelayCmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()