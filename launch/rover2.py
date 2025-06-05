#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        
        # Создаем QoS профиль для трансформ
        tf_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Подписка на реальные данные из Gazebo
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        
        # Публикация TF
        self.tf_broadcaster = TransformBroadcaster(self, qos=tf_qos)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self, qos=tf_qos)
        
        # Инициализация переменных
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_angles = {
            'left_wheel_joint': 0.0,
            'right_wheel_joint': 0.0,
            'back_left_wheel_joint': 0.0,
            'back_right_wheel_joint': 0.0
        }
        
        # Публикация статических трансформ
        self.publish_static_transforms()
        
        # Таймер для публикации динамических трансформ
        self.create_timer(0.05, self.publish_dynamic_transforms)
        
        self.get_logger().info("Ровер полностью синхронизирован! Точность обеспечена!")

    def odom_callback(self, msg):
        """Обработка реальной одометрии из Gazebo"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Конвертация кватерниона в угол рысканья (yaw)
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def joint_states_callback(self, msg):
        """Обработка реальных углов колёс из Gazebo"""
        for i, name in enumerate(msg.name):
            if name in self.wheel_angles:
                self.wheel_angles[name] = msg.position[i]

    def publish_static_transforms(self):
        """Публикация статических трансформ (один раз)"""
        transforms = []
        
        # Шасси (chassis) - исправлено смещение
        t_chassis = TransformStamped()
        t_chassis.header.stamp = self.get_clock().now().to_msg()
        t_chassis.header.frame_id = 'base_link'
        t_chassis.child_frame_id = 'chassis'
        t_chassis.transform.translation.x = 0.0  # Было -0.1
        t_chassis.transform.translation.y = 0.0
        t_chassis.transform.translation.z = 0.0
        t_chassis.transform.rotation.w = 1.0
        transforms.append(t_chassis)
        
        self.static_tf_broadcaster.sendTransform(transforms)

    def publish_dynamic_transforms(self):
        """Публикация динамических трансформ (вызывается по таймеру)"""
        transforms = []
        now = self.get_clock().now().to_msg()
        
        # Трансформация odom → base_link
        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = self.x
        t_base.transform.translation.y = self.y
        t_base.transform.translation.z = 0.0
        
        # Конвертация угла в кватернион
        q = quaternion_from_euler(0, 0, self.theta)
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        transforms.append(t_base)
        
        # Трансформы колёс (base_link → wheel) - ИСПРАВЛЕНА ОРИЕНТАЦИЯ!
        wheel_positions = {
            'left_wheel_joint': (-0.38, 0.38, 0.0),
            'right_wheel_joint': (-0.38, -0.38, 0.0),
            'back_left_wheel_joint': (0.48, 0.38, 0.0),
            'back_right_wheel_joint': (0.48, -0.38, 0.0)
        }
        
        # КОРРЕКТНАЯ ориентация колёс (без лишнего поворота)
        for wheel_name in wheel_positions.keys():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_link'
            t.child_frame_id = wheel_name.replace('_joint', '')  # left_wheel_joint → left_wheel
            
            pos = wheel_positions[wheel_name]
            t.transform.translation.x = pos[0]
            t.transform.translation.y = pos[1]
            t.transform.translation.z = pos[2]
            
            # ТОЛЬКО вращение вокруг оси Z (вертикальная ориентация)
            angle = self.wheel_angles[wheel_name]
            q_rot = quaternion_from_euler(0, 0, angle)
            
            t.transform.rotation.x = q_rot[0]
            t.transform.rotation.y = q_rot[1]
            t.transform.rotation.z = q_rot[2]
            t.transform.rotation.w = q_rot[3]
            
            transforms.append(t)
        
        self.tf_broadcaster.sendTransform(transforms)

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