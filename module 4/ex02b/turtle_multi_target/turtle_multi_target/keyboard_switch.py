import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import select
import sys
import threading


class KeyboardSwitch(Node):
    def __init__(self):
        super().__init__('keyboard_switch')
        
        # Publisher для отправки команд переключения
        self.switch_pub = self.create_publisher(String, '/keyboard_switch', 10)
        
        self.get_logger().info('Keyboard switch node started')
        self.get_logger().info('Нажимайте "n" для переключения цели, "q" для выхода')
        
        # Запускаем поток для чтения клавиатуры
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def keyboard_listener(self):
        """Поток для чтения клавиатуры"""
        try:
            while rclpy.ok():
                # Проверяем ввод без блокировки
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.readline().strip().lower()
                    if key == 'n':
                        self.get_logger().info('Обнаружено нажатие "n" - переключаем цель')
                        msg = String()
                        msg.data = 'switch'
                        self.switch_pub.publish(msg)
                    elif key == 'q':
                        self.get_logger().info('Выход')
                        break
        except Exception as e:
            self.get_logger().error(f'Ошибка чтения клавиатуры: {e}')


def main():
    rclpy.init()
    node = KeyboardSwitch()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
