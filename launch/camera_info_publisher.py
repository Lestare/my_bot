#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.publisher = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Параметры камеры (пример для Gazebo)
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'camera_link'
        self.camera_info.height = 480
        self.camera_info.width = 640
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Искажение
        self.camera_info.k = [
            554.254691191187, 0.0, 320.5,
            0.0, 554.254691191187, 240.5,
            0.0, 0.0, 1.0
        ]
        self.camera_info.p = [
            554.254691191187, 0.0, 320.5, 0.0,
            0.0, 554.254691191187, 240.5, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        self.get_logger().info("Camera Info Publisher Started")

    def timer_callback(self):
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()