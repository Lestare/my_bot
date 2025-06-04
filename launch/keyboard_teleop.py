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
    
    def _update_motors(self):
        print(f"\rЛевые: {self.left_speed:4}% | Правые: {self.right_speed:4%}", end='')
        sys.stdout.flush()

    def print_instructions(self):
        print("Управление ровером (WASD):")
        print("W - вперед, S - назад, A - влево, D - вправо")
        print("Пробел - остановка, Q - выход")
        print("Комбинации: W+A, W+D, S+A, S+D")
    
    def get_key(timeout=0.1):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return None
    
    def update_twist(self):
        twist = Twist()
        
        try:
            while True:
                key = self.get_key()
                if key is None:
                    continue
                    
                key = key.lower()
                left = self.angular_speed.left_speed
                right = self.angular_speed.right_speed
                
                # Одиночные команды
                if key == 'w':
                    left = self.linear_speed
                    right = self.linear_speed
                elif key == 's':
                    left = -self.linear_speed
                    right = -self.linear_speed
                elif key == 'a':
                    left = -self.angular_speed
                    right = self.angular_speed
                elif key == 'd':
                    left = self.angular_speed
                    right = -self.angular_speed
                elif key == ' ':
                    left = 0
                    right = 0
                elif key == 'q':
                    break
                    
                # Комбинированные команды (проверяем следующую клавишу без ожидания)
                if key in ('w', 's'):
                    next_key = self.get_key(0)
                    if next_key:
                        next_key = next_key.lower()
                        if next_key == 'a':  # Плавный поворот налево
                            if key == 'w':
                                left = self.linear_speed - self.angular_speed
                                right = self.linear_speed
                            else:  # s
                                left = -self.linear_speed + self.angular_speed
                                right = -self.linear_speed
                        elif next_key == 'd':  # Плавный поворот направо
                            if key == 'w':
                                left = self.linear_speed
                                right = self.linear_speed - self.angular_speed
                            else:  # s
                                left = -self.linear_speed
                                right = -self.linear_speed + self.angular_speed
                
                self.angular_speed.set_speeds(left, right)
                
        except KeyboardInterrupt:
            pass
        finally:
           self.linear_speed = 0
           self.angular_speed = 0
                        
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