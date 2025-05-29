import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        
        # Слушаем команды с пульта
        self.create_subscription(Twist, 'remote_cmd', self.cmd_callback, 10)
        
        # Отправляем одометрию на пульт
        self.odom_pub = self.create_publisher(Odometry, 'remote_odom', 10)
        
        # Слушаем свою одометрию
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        print("Ровер готов! Жду команд...")

    def cmd_callback(self, msg):
        # Публикуем команды в моторы
        motor_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        motor_pub.publish(msg)
        print(f"Принял команду: вперед={msg.linear.x}, поворот={msg.angular.z}")

    def odom_callback(self, msg):
        # Отправляем положение на пульт
        self.odom_pub.publish(msg)

def main():
    rclpy.init()
    node = Rover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()