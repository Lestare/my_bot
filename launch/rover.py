#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
import math

class Rover(Node):
    def __init__(self):
        super().__init__('rover_odom')
        
        # Параметры робота
        self.wheel_separation = 0.76
        self.wheel_radius = 0.17
        
        # Состояние
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Подписка на команды колес
        self.create_subscription(Twist, 'wheel_velocities', self.velocity_callback, 10)
        
        # Публикация одометрии
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Одометрия ровера запущена")

    def velocity_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        # Рассчет кинематики для 4 колес
        vx = (msg.linear.x + msg.linear.y) / 2 * self.wheel_radius
        vy = 0.0
        vth = (msg.linear.y - msg.linear.x) / self.wheel_separation * self.wheel_radius
        
        # Интегрирование
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_th = vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th
        
        # Публикация TF и Odometry
        self.publish_odometry(now)
        self.last_time = now

    def publish_odometry(self, now):
        # TF трансформация
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(t)
        
        # Odometry сообщение
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = RoverOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()