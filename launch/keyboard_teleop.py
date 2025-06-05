#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios
import select

class Teleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Параметры скорости
        self.linear_speed = 0.5   # м/с
        self.angular_speed = 0.8  # рад/с
        
        # Состояние клавиш
        self.key_states = {
            'w': False, 
            's': False,
            'a': False,
            'd': False,
            'q': False,
            'e': False,
            'z': False,
            'c': False,
            'r': False,
            'f': False,
            ' ': False
        }
        
        # Настройки терминала
        self.settings = termios.tcgetattr(sys.stdin)
        self.print_instructions()

    def print_instructions(self):
        print("Управление (зажимайте клавиши):")
        print("  W - вперед")
        print("  S - назад")
        print("  A - влево")
        print("  D - вправо")
        print(" Q - вперед+влево")
        print(" E - вперед+вправо")
        print(" Z - назад+влево")
        print(" C - назад+вправо")
        print(" R - увеличение")
        print(" F - уменьшение")
        print("  Space - стоп")
        print("  Ctrl+C - выход")

    def update_twist(self):
        twist = Twist()
        
        # Обработка движения вперед/назад
        if self.key_states['w'] and not self.key_states['s']:
            twist.linear.x = self.linear_speed
        elif self.key_states['s'] and not self.key_states['w']:
            twist.linear.x = -self.linear_speed
        
        # Обработка поворотов
        if self.key_states['a'] and not self.key_states['d']:
            twist.angular.z = self.angular_speed
        elif self.key_states['d'] and not self.key_states['a']:
            twist.angular.z = -self.angular_speed
        
        # Обработка движения вперед/назад c поворотом влево
        if self.key_states['q']:
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
        if self.key_states['z']:
            twist.linear.x = -self.linear_speed
            twist.angular.z = self.angular_speed
        
        # Обработка движения вперёд/назад с поворотом вправо
        if self.key_states['e']:
            twist.linear.x = self.linear_speed
            twist.angular.z = -self.angular_speed
        if self.key_states['c']:
            twist.linear.x = -self.linear_speed
            twist.angular.z = -self.angular_speed
        
        # Обработка движения вперёд/назад с поворотом вправо
        if self.key_states['r']:
            self.linear_speed = self.linear_speed * 1.10
            self.angular_speed = self.angular_speed * 1.10
        if self.key_states['f']:
            self.linear_speed = self.linear_speed / 1.10
            self.angular_speed = self.angular_speed / 1.10

        # Остановка по пробелу
        if self.key_states[' ']:
            twist = Twist()
        
        return twist

    def run(self):
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                # Проверяем доступность ввода без блокировки
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    # Выход по Ctrl+C
                    if key == '\x03':
                        break
                    
                    # Обновляем состояние клавиш
                    if key in self.key_states:
                        self.key_states[key] = True
                 
                # Отправляем команду движения
                twist = self.update_twist()
                self.publisher_.publish(twist)
                
                # Сбрасываем состояние клавиш (кроме пробела)
                for k in ['w', 's', 'a', 'd', 'q', 'e', 'z', 'c', 'r', 'f' , ' ']:
                    self.key_states[k] = False
                
                # Небольшая задержка для CPU
                rclpy.spin_once(self, timeout_sec=0.01)
                
        finally:
            # Восстанавливаем настройки терминала
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            # Останавливаем ровер перед выходом
            stop_twist = Twist()
            self.publisher_.publish(stop_twist)
            self.get_logger().info("Ровер остановлен")

def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    teleop.run()
    teleop.destroy_node()
main()