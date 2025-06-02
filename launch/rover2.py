#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String  # Для публикации URDF

class RoverTF(Node):
    def __init__(self):
        super().__init__('rover_tf')
        
        # Подписка на команды
        self.create_subscription(Twist, 'remote_cmd', self.cmd_callback, 10)
        
        # Публикация TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Публикация URDF (важно для RViz2)
        self.urdf_pub = self.create_publisher(String, '/robot_description', 10)
        
        # Имитация положения
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Таймеры
        self.create_timer(0.1, self.publish_tf)
        self.create_timer(1.0, self.publish_urdf)  # Публикуем URDF раз в секунду
        
        self.get_logger().info("Ровер TF готов!")

    def publish_urdf(self):
        """Публикация простого URDF для визуализации"""
        urdf = """
        <robot name="simple_rover">
            <link name="base_link">
                <visual>
                    <geometry>
                        <box size="0.4 0.2 0.1"/>
                    </geometry>
                    <material name="blue">
                        <color rgba="0 0 1 1"/>
                    </material>
                </visual>
            </link>
        </robot>
        """
        msg = String()
        msg.data = urdf
        self.urdf_pub.publish(msg)

    def cmd_callback(self, msg):
        # Обновляем положение на основе команд
        self.theta += msg.angular.z * 0.1
        self.x += msg.linear.x * math.cos(self.theta) * 0.1
        self.y += msg.linear.x * math.sin(self.theta) * 0.1
        self.get_logger().info(f"Позиция: X={self.x:.2f}, Y={self.y:.2f}", throttle_duration_sec=1)

    def publish_tf(self):
        # Создаем трансформацию
        t = TransformStamped()
        
        # Заполняем заголовок
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Родительская система координат
        t.child_frame_id = 'base_link'  # Система координат ровера
        
        # Позиция
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Ориентация (из угла в кватернион)
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        # Публикуем трансформацию
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = RoverTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()