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
        """Публикация правильного URDF"""
        urdf = """
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="rover">

    <material name="white">
        <color rgba="1 1 1 1"/>
    </material>

    <material name="orange">
        <color rgba="1 0.3 0.1 1"/>
    </material>

    <material name="blue">
        <color rgba="0.2 0.2 1 1"/>
    </material>

    <material name="black">
        <color rgba="0 0 0 1"/>
    </material>

    <link name="base_link"/>

    <joint name="chassis_joint" type="fixed">
        <parent link="base_link"/>
        <child link="chassis"/>
        <origin xyz="-0.1 0 0"/>
    </joint>

    <link name="chassis">
        <visual>
            <origin xyz="0.15 0 0.075" rpy="0 0 0"/>
            <geometry>
                <box size="1.2 0.6 0.26"/>
            </geometry>
            <material name="white"/>
        </visual>
    </link>
    
    <!-- LEFT WHEEL -->

    <joint name="left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
        <origin xyz="-0.38 0.38 0" rpy="-1.5708 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>

    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder length="0.15" radius="0.17"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>
 
    <!-- RIGHT WHEEL -->

    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="-0.38 -0.38 0" rpy="1.5708 0 0"/>
        <axis xyz="0 0 -1"/>
    </joint>

    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder length="0.15" radius="0.17"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <!-- BACK LEFT WHEEL -->

    <joint name="back_left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="back_left_wheel"/>
        <origin xyz="0.48 0.38 0" rpy="-1.5708 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>

    <link name="back_left_wheel">
        <visual>
            <geometry>
                <cylinder length="0.15" radius="0.17"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <!-- BACK RIGHT WHEEL -->

    <joint name="back_right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="back_right_wheel"/>
        <origin xyz="0.48 -0.38 0" rpy="1.5708 0 0"/>
        <axis xyz="0 0 -1"/>
    </joint>

    <link name="back_right_wheel">
        <visual>
            <geometry>
                <cylinder length="0.15" radius="0.17"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

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
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = Rover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()