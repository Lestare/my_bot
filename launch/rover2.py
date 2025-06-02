#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        
        # Подписка на команды
        self.create_subscription(Twist, 'remote_cmd', self.cmd_callback, 10)
        
        # Публикация TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Публикация Odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Публикация URDF с правильным QoS
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.urdf_pub = self.create_publisher(String, '/robot_description', qos_profile)
        
        # Имитация положения
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Таймеры
        self.create_timer(0.05, self.publish_transforms)  # 20 Гц
        self.create_timer(1.0, self.publish_urdf)  # Публикуем URDF раз в секунду
        
        # Публикация URDF при старте
        self.publish_urdf()
        
        self.get_logger().info("Ровер готов к работе!")

    def publish_urdf(self):
        """Публикация правильного URDF с колесами"""
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
            
            <!-- Добавляем колеса -->
            <link name="wheel_left">
                <visual>
                    <geometry>
                        <cylinder length="0.05" radius="0.08"/>
                    </geometry>
                    <material name="black">
                        <color rgba="0.1 0.1 0.1 1"/>
                    </material>
                </visual>
            </link>
            
            <link name="wheel_right">
                <visual>
                    <geometry>
                        <cylinder length="0.05" radius="0.08"/>
                    </geometry>
                    <material name="black">
                        <color rgba="0.1 0.1 0.1 1"/>
                    </material>
                </visual>
            </link>
            
            <!-- Соединяем колеса с основой -->
            <joint name="wheel_left_joint" type="continuous">
                <parent link="base_link"/>
                <child link="wheel_left"/>
                <origin xyz="0.2 0.15 -0.1" rpy="1.57 0 0"/>
                <axis xyz="0 1 0"/>
            </joint>
            
            <joint name="wheel_right_joint" type="continuous">
                <parent link="base_link"/>
                <child link="wheel_right"/>
                <origin xyz="0.2 -0.15 -0.1" rpy="1.57 0 0"/>
                <axis xyz="0 1 0"/>
            </joint>
        </robot>
        """
        msg = String()
        msg.data = urdf
        self.urdf_pub.publish(msg)
        self.get_logger().info("URDF опубликован")

    def cmd_callback(self, msg):
        # Обновляем положение на основе команд
        self.theta += msg.angular.z * 0.05
        self.x += msg.linear.x * math.cos(self.theta) * 0.05
        self.y += msg.linear.x * math.sin(self.theta) * 0.05
        self.get_logger().info(f"Позиция: X={self.x:.2f}, Y={self.y:.2f}")

    def publish_transforms(self):
        # Создаем трансформацию для основания
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = self.x
        t_base.transform.translation.y = self.y
        t_base.transform.translation.z = 0.0
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        t_base.transform.rotation.z = sy
        t_base.transform.rotation.w = cy
        
        # Публикуем трансформацию
        self.tf_broadcaster.sendTransform(t_base)
        
        # Публикация Odometry
        odom = Odometry()
        odom.header.stamp = t_base.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = Rover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()