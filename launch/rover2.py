#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        
        # Отключаем TF публикацию - Gazebo делает это сам
        self.get_logger().warning("Ровер работает в режиме только статических трансформ")
        
        # Создаем QoS профиль для трансформ
        tf_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Публикация TF только для статических элементов
        self.static_tf_broadcaster = StaticTransformBroadcaster(self, qos=tf_qos)
        
        # Публикация статических трансформ
        self.publish_static_transforms()
        
        # Мониторинг данных
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        
        self.get_logger().info("Ровер синхронизирован! Используются TF из Gazebo")

    def odom_callback(self, msg):
        """Мониторинг одометрии"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"Одометрия: X={x:.2f}, Y={y:.2f}", throttle_duration_sec=1)

    def joint_states_callback(self, msg):
        """Мониторинг состояний шарниров"""
        for i, name in enumerate(msg.name):
            angle = msg.position[i]
            self.get_logger().info(f"Шарнир {name}: {angle:.2f} рад", throttle_duration_sec=1)

    def publish_static_transforms(self):
        """Публикация ТОЛЬКО статических трансформ"""
        transforms = []
        
        # Шасси (chassis)
        t_chassis = TransformStamped()
        t_chassis.header.stamp = self.get_clock().now().to_msg()
        t_chassis.header.frame_id = 'base_link'
        t_chassis.child_frame_id = 'chassis'
        t_chassis.transform.translation.x = 0.0
        t_chassis.transform.translation.y = 0.0
        t_chassis.transform.translation.z = 0.0
        t_chassis.transform.rotation.w = 1.0
        transforms.append(t_chassis)
        
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info("Статические трансформы опубликованы")

def main():
    rclpy.init()
    node = Rover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
