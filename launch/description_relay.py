#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import subprocess

class DescriptionRelay(Node):
    def __init__(self):
        super().__init__('description_relay')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/robot_description', qos)
        self.declare_parameter('urdf_path', '')
        
        urdf_path = self.get_parameter('urdf_path').value
        if not urdf_path:
            self.get_logger().error("URDF path not provided!")
            return
        
        # Конвертируем xacro в URDF
        try:
            result = subprocess.run(
                ['xacro', urdf_path],
                capture_output=True,
                text=True,
                check=True
            )
            self.urdf = result.stdout
        except Exception as e:
            self.get_logger().error(f"Failed to process URDF: {str(e)}")
            return
        
        self.publish_description()
        self.create_timer(5.0, self.publish_description)
        self.get_logger().info("URDF relay ready")

    def publish_description(self):
        msg = String()
        msg.data = self.urdf
        self.pub.publish(msg)
        self.get_logger().info("Published URDF", once=True)

def main():
    rclpy.init()
    node = DescriptionRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()