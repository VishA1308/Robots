import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        self.switch_threshold = self.declare_parameter('switch_threshold', 1.0).get_parameter_value().double_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.current_target = 'carrot1'
        self.last_switch_time = self.get_clock().now()
        self.switch_cooldown = 2.0  # 2 секунды задержки между переключениями
        
        # Создаем publisher для команд движения
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        # Создаем publisher для информации о целях
        self.target_info_pub = self.create_publisher(String, '/current_target', 10)
        
        # Создаем publisher для переключения целей
        self.target_switch_pub = self.create_publisher(String, '/switch_target', 10)
        
        # Подписываемся на переключение целей от target_switcher
        self.target_sub = self.create_subscription(
            String,
            '/current_target_name',
            self.target_callback,
            10)
        
        # Таймер для основного цикла управления
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Turtle controller started')
        self.get_logger().info(f'Following target: {self.current_target}')

    def target_callback(self, msg):
        """Обработчик смены цели от target_switcher"""
        if msg.data in self.targets:
            old_target = self.current_target
            self.current_target = msg.data
            self.last_switch_time = self.get_clock().now()
            self.get_logger().info(f'Target changed: {old_target} -> {self.current_target}')

    def get_next_target(self):
        """Получить следующую цель в цикле"""
        current_index = self.targets.index(self.current_target)
        return self.targets[(current_index + 1) % len(self.targets)]

    def control_loop(self):
        """Основной цикл управления"""
        try:
            # Получаем трансформацию от turtle2 к текущей цели
            transform = self.tf_buffer.lookup_transform(
                'turtle2',
                self.current_target,
                rclpy.time.Time())
            
            # Вычисляем расстояние и угол
            dx = transform.transform.translation.x
            dy = transform.transform.translation.y
            distance = math.sqrt(dx*dx + dy*dy)
            angle = math.atan2(dy, dx)
            
            # Публикуем информацию о цели
            target_info = f"name: {self.current_target}, x: {dx:.2f}, y: {dy:.2f}, distance: {distance:.2f}"
            info_msg = String()
            info_msg.data = target_info
            self.target_info_pub.publish(info_msg)
            
            # Проверяем автоматическое переключение (только если прошло достаточно времени с последнего переключения)
            current_time = self.get_clock().now()
            time_since_switch = (current_time - self.last_switch_time).nanoseconds / 1e9
            
            if distance < self.switch_threshold and time_since_switch > self.switch_cooldown:
                next_target = self.get_next_target()
                self.get_logger().info(f'Auto-switching from {self.current_target} to {next_target} (distance: {distance:.2f})')
                
                # Публикуем команду переключения через наш публикатор
                switch_msg = String()
                switch_msg.data = next_target
                self.target_switch_pub.publish(switch_msg)
                return  # Пропускаем управление в этом цикле
            
            # Управление turtle2
            cmd_vel = Twist()
            
            # Более плавное управление с ограничениями
            linear_speed = 0.5 * min(distance, 2.0)
            angular_speed = 2.0 * angle
            
            # Ограничиваем максимальную угловую скорость
            max_angular = 1.5
            if abs(angular_speed) > max_angular:
                angular_speed = max_angular * (1 if angular_speed > 0 else -1)
            
            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = angular_speed
            
            self.cmd_vel_pub.publish(cmd_vel)
            
        except TransformException as e:
            self.get_logger().warn(f'Transform error: {e}', throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')


def main():
    rclpy.init()
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
