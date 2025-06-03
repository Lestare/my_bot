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
                for k in ['w', 's', 'a', 'd']:
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
    rclpy.shutdown()

if __name__ == '__main__':
    main()