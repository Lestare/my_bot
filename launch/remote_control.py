#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RemoteOperator(Node):
    def __init__(self):
        super().__init__('remote_operator')
        
        # Публикация команд на ровер
        self.cmd_pub = self.create_publisher(Twist, 'remote_cmd_vel', 10)
        
        # Подписка на одометрию от ровера
        self.odom_sub = self.create_subscription(
            Odometry,
            'remote_odom',
            self.odom_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.send_command)
        self.get_logger().info("Remote Operator: Ready to send commands")

    def send_command(self):
        # Вручную задаем команды (замените на свой источник команд)
        cmd = Twist()
        cmd.linear.x = 0.5  # Пример: движение вперед
        cmd.angular.z = 0.2 # Пример: поворот
        self.cmd_pub.publish(cmd)

    def odom_callback(self, msg):
        # Отображение данных одометрии
        pos = msg.pose.pose.position
        self.get_logger().info(f"Rover Position: x={pos.x:.2f}, y={pos.y:.2f}")

def main():
    rclpy.init()
    node = RemoteOperator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()