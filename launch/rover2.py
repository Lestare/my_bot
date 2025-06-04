#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf_transformations import quaternion_from_euler, quaternion_multiply

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        
        # Подписка на команды
        self.create_subscription(Twist, 'remote_cmd', self.cmd_callback, 10)
        
        # Публикация TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Публикация Odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Имитация положения
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Углы вращения колёс
        self.wheel_angles = {
            'left_wheel': 0.0,
            'right_wheel': 0.0,
            'back_left_wheel': 0.0,
            'back_right_wheel': 0.0
        }
        
        # Параметры ровера
        self.WHEEL_RADIUS = 0.17
        self.TRACK_WIDTH = 0.76  # Расстояние между левыми и правыми колёсами
        
        # Публикация статических трансформ (один раз при старте)
        self.publish_static_transforms()
        
        # Таймеры
        self.create_timer(0.05, self.publish_dynamic_transforms)  # 20 Гц
        
        self.get_logger().info("Ровер готов к работе!")

    def publish_static_transforms(self):
        """Публикация статических трансформ (неизменные части)"""
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
        """Публикация динамических трансформ (изменяющиеся части)"""
        transforms = []
        now = self.get_clock().now().to_msg()
        
        # 1. Трансформа для основания (odom → base_link)
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
        
        # 2. Трансформы для колёс (base_link → wheel)
        wheel_positions = {
            'left_wheel': (-0.38, 0.38, 0.0),
            'right_wheel': (-0.38, -0.38, 0.0),
            'back_left_wheel': (0.48, 0.38, 0.0),
            'back_right_wheel': (0.48, -0.38, 0.0)
        }
        
        # Базовые ориентации колёс из URDF (в RPY: roll, pitch, yaw)
        wheel_base_orientations = {
            'left_wheel': (-math.pi/2, 0, 0),
            'right_wheel': (math.pi/2, 0, 0),
            'back_left_wheel': (-math.pi/2, 0, 0),
            'back_right_wheel': (math.pi/2, 0, 0)
        }
        
        for wheel_name in self.wheel_angles:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_link'
            t.child_frame_id = wheel_name
            
            # Позиция из URDF
            x, y, z = wheel_positions[wheel_name]
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            
            # Базовый кватернион ориентации из URDF
            roll, pitch, yaw = wheel_base_orientations[wheel_name]
            q_base = quaternion_from_euler(roll, pitch, yaw)
            
            # Дополнительное вращение вокруг оси Z (для левых колёс) или -Z (для правых)
            rotation_angle = self.wheel_angles[wheel_name]
            if 'left' in wheel_name:
                q_rotation = quaternion_from_euler(0, 0, rotation_angle)
            else:  # Для правых колёс
                q_rotation = quaternion_from_euler(0, 0, -rotation_angle)
            
            # Комбинируем ориентации: сначала базовый поворот, затем вращение колеса
            q_total = quaternion_multiply(q_base, q_rotation)
            
            t.transform.rotation.x = q_total[0]
            t.transform.rotation.y = q_total[1]
            t.transform.rotation.z = q_total[2]
            t.transform.rotation.w = q_total[3]
            
            transforms.append(t)
        
        # Публикуем все трансформы
        self.tf_broadcaster.sendTransform(transforms)
        
        # 3. Публикация Odometry
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
        dt = 0.05  # Период обновления
        
        # Обновляем положение основания
        self.theta += msg.angular.z * dt
        self.x += msg.linear.x * math.cos(self.theta) * dt
        self.y += msg.linear.x * math.sin(self.theta) * dt
        
        # Расчет скоростей для каждого колеса
        left_speed = (msg.linear.x - msg.angular.z * self.TRACK_WIDTH / 2) / self.WHEEL_RADIUS
        right_speed = (msg.linear.x + msg.angular.z * self.TRACK_WIDTH / 2) / self.WHEEL_RADIUS
        
        # Обновляем углы вращения колёс
        self.wheel_angles['left_wheel'] += left_speed * dt
        self.wheel_angles['right_wheel'] += right_speed * dt
        self.wheel_angles['back_left_wheel'] += left_speed * dt
        self.wheel_angles['back_right_wheel'] += right_speed * dt
        
        # Логирование с ограничением частоты (1 раз в секунду)
        self.get_logger().info(f"Позиция: X={self.x:.2f}, Y={self.y:.2f}", throttle_duration_sec=1)

def euler_to_quaternion(roll, pitch, yaw):
    """Конвертация RPY в кватернион (w, x, y, z)"""
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
    
    return (w, x, y, z)

def quaternion_multiply(q1, q2):
    """Умножение двух кватернионов (w, x, y, z)"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (w, x, y, z)

def quaternion_from_euler(roll, pitch, yaw):
    """Аналог tf_transformations.quaternion_from_euler"""
    return euler_to_quaternion(roll, pitch, yaw)

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