#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy

class DescriptionRelay(Node):
    def __init__(self):
        super().__init__('description_relay')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/robot_description', qos)
        
        # Читаем URDF из файла
        with open('/path/to/your/robot.urdf', 'r') as f:
            self.urdf = f.read()
        
        # Публикуем сразу и периодически
        self.publish_description()
        self.create_timer(5.0, self.publish_description)
        self.get_logger().info("URDF relay ready")

    def publish_description(self):
        msg = String()
        msg.data = self.urdf
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = DescriptionRelay()
    rclpy.spin(node)
    rclpy.shutdown()