#!/usr/bin/env python

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Функция обратного вызова для получения информации о перемещении
def odometry_callback(data):
    # Извлечение необходимой информации о перемещении
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    rclpy.loginfo("Current Position: x=%.2f, y=%.2f, z=%.2f", position.x, position.y, position.z)
    rclpy.loginfo("Current Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", orientation.x, orientation.y, orientation.z, orientation.w)

    # Здесь вы можете добавить код для отправки данных на серверный компьютер
    # Например, используя сокеты или HTTP-запросы.

def movement_publisher():
    # Инициализация узла ROS
    rclpy.init_node('movement_publisher', anonymous=True)

    # Подписка на тему одометрии
    rclpy.Subscriber("odom", Odometry, odometry_callback)

    # Публикация команды движения
    cmd_pub = rclpy.Publisher("cmd_vel", Twist, queue_size=10)

    rate = rclpy.Rate(10)  # 10 Гц
    while not rclpy.is_shutdown():
        cmd_msg = Twist()
        # Пример команды движения
        cmd_msg.linear.x = 0.5  # Скорость вперед
        cmd_msg.angular.z = 0.0  # Никакого вращения

        cmd_pub.publish(cmd_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        movement_publisher()
    except rclpy.ROSInterruptException:
        pass
