#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf_transformations import euler_from_quaternion

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
        
        # Подписка на реальную одометрию из Gazebo
        self.create_subscription(Odometry, 'gazebo_odom', self.gazebo_odom_callback, 10)
        
        # Подписка на состояния шарниров
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        
        # Публикация TF
        self.tf_broadcaster = TransformBroadcaster(self, qos=tf_qos)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self, qos=tf_qos)
        
        # Публикация Odometry
        odom_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', odom_qos)
        
        # Актуальные данные из Gazebo
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_angles = {
            'left_wheel': 0.0,
            'right_wheel': 0.0,
            'back_left_wheel': 0.0,
            'back_right_wheel': 0.0
        }
        
        # Публикация статических трансформ
        self.publish_static_transforms()
        
        # Таймер
        self.create_timer(0.05, self.publish_dynamic_transforms)
        
        self.get_logger().info("Ровер синхронизирован с Gazebo!")

    def gazebo_odom_callback(self, msg):
        """Получение реального положения из Gazebo"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Конвертация кватерниона в угол
        q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def joint_states_callback(self, msg):
        """Получение реальных углов колёс из Gazebo"""
        for i, name in enumerate(msg.name):
            if name in self.wheel_angles:
                self.wheel_angles[name] = msg.position[i]

    def publish_static_transforms(self):
        """Публикация статических трансформ"""
        transforms = []
        
        # Шасси (chassis)
        t_chassis = TransformStamped()
        t_chassis.header.stamp = self.get_clock().now().to_msg()
        t_chassis.header.frame_id = 'base_link'
        t_chassis.child_frame_id = 'chassis'
        t_chassis.transform.translation.x = -0.1
        t_chassis.transform.translation.y = 0.0
        t_chassis.transform.translation.z = 0.0
        t_chassis.transform.rotation.w = 1.0
        transforms.append(t_chassis)
        
        self.static_tf_broadcaster.sendTransform(transforms)

    def publish_dynamic_transforms(self):
        """Публикация динамических трансформ"""
        transforms = []
        now = self.get_clock().now().to_msg()
        
        # Трансформа основания (odom → base_link)
        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = self.x
        t_base.transform.translation.y = self.y
        t_base.transform.translation.z = 0.0
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        t_base.transform.rotation.z = sy
        t_base.transform.rotation.w = cy
        transforms.append(t_base)
        
        # Трансформы колёс
        wheel_positions = {
            'left_wheel': (-0.38, 0.38, 0.0),
            'right_wheel': (-0.38, -0.38, 0.0),
            'back_left_wheel': (0.48, 0.38, 0.0),
            'back_right_wheel': (0.48, -0.38, 0.0)
        }
        
        # Базовые ориентации из URDF
        wheel_base_orientations = {
            'left_wheel': self.euler_to_quaternion(-math.pi/2, 0, 0),
            'right_wheel': self.euler_to_quaternion(-math.pi/2, 0, 0),
            'back_left_wheel': self.euler_to_quaternion(-math.pi/2, 0, 0),
            'back_right_wheel': self.euler_to_quaternion(-math.pi/2, 0, 0)
        }
        
        for wheel_name, pos in wheel_positions.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_link'
            t.child_frame_id = wheel_name
            t.transform.translation.x = pos[0]
            t.transform.translation.y = pos[1]
            t.transform.translation.z = pos[2]
            
            # Базовый кватернион ориентации
            q_base = wheel_base_orientations[wheel_name]
            
            # Вращение колеса
            angle = self.wheel_angles[wheel_name]
            q_rot = self.euler_to_quaternion(0, angle, 0)  # Вращение вокруг оси Y
            
            # Комбинируем ориентации
            q_total = self.quaternion_multiply(q_base, q_rot)
            
            t.transform.rotation.x = q_total[0]
            t.transform.rotation.y = q_total[1]
            t.transform.rotation.z = q_total[2]
            t.transform.rotation.w = q_total[3]
            
            transforms.append(t)
        
        self.tf_broadcaster.sendTransform(transforms)
        
        # Публикация Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        self.odom_pub.publish(odom)

    def cmd_callback(self, msg):
        """Обработка команд движения"""
        dt = 0.05
        
        # Обновление положения
        self.theta += msg.angular.z * dt
        self.x += msg.linear.x * math.cos(self.theta) * dt
        self.y += msg.linear.x * math.sin(self.theta) * dt
        
        # Расчет скоростей колёс
        left_speed = (msg.linear.x - msg.angular.z * self.TRACK_WIDTH / 2) / self.WHEEL_RADIUS
        right_speed = (msg.linear.x + msg.angular.z * self.TRACK_WIDTH / 2) / self.WHEEL_RADIUS
        
        # Обновление углов вращения
        self.wheel_angles['left_wheel'] += left_speed * dt
        self.wheel_angles['right_wheel'] += right_speed * dt
        self.wheel_angles['back_left_wheel'] += left_speed * dt
        self.wheel_angles['back_right_wheel'] += right_speed * dt
        
        self.get_logger().info(f"Position: X={self.x:.2f}, Y={self.y:.2f}", throttle_duration_sec=1)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Конвертация RPY в кватернион (x, y, z, w)"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)

    def quaternion_multiply(self, q1, q2):
        """Умножение кватернионов"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return (x, y, z, w)

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