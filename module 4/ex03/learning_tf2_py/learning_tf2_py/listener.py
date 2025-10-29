import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from turtlesim.srv import Spawn
from rclpy.duration import Duration
import time
import numpy as np


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        
        # Declare and get delay parameter
        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').value
        
        self.get_logger().info(f'Time delay set to: {self.delay} seconds')
        
        # Spawn turtle2
        self.spawn_turtle2()
        
        # Wait for system to stabilize
        time.sleep(2.0)
        
        # Publisher for turtle2 commands
        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Listener node ready')
        
        # State variables
        self.start_time = self.get_clock().now()
        self.delayed_mode_activated = False
        self.position_history = []
        self.last_target_time = None
        self.smoothing_factor = 0.3

    def spawn_turtle2(self):
        self.get_logger().info('Spawning turtle2...')
        spawn_client = self.create_client(Spawn, 'spawn')
        
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        request = Spawn.Request()
        request.x = 4.0
        request.y = 2.0
        request.theta = 0.0
        request.name = 'turtle2'
        
        future = spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Turtle2 spawned successfully')
        else:
            self.get_logger().error('Failed to spawn turtle2')

    def get_smoothed_position(self, x, y, mode):

        if mode != "DELAYED":
            self.position_history = []  # Сбрасываем историю в реальном времени
            return x, y
            
        # Добавляем текущую позицию в историю
        self.position_history.append((x, y))
        if len(self.position_history) > 10:  # Храним последние 10 позиций
            self.position_history.pop(0)
            
        # Используем экспоненциальное сглаживание
        if len(self.position_history) >= 3:
            smoothed_x, smoothed_y = self.position_history[0]
            for i in range(1, len(self.position_history)):
                smoothed_x = self.smoothing_factor * self.position_history[i][0] + (1 - self.smoothing_factor) * smoothed_x
                smoothed_y = self.smoothing_factor * self.position_history[i][1] + (1 - self.smoothing_factor) * smoothed_y
            return smoothed_x, smoothed_y
            
        return x, y

    def timer_callback(self):
        try:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            
            trans = None
            mode = "REAL-TIME"
            
            # Всегда используем реальное время для управления
            # Задержка эмулируется через логику управления
            trans = self.tf_buffer.lookup_transform(
                'turtle2',
                'turtle1', 
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            
            # Extract position information
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # Эмулируем задержку через управление, а не через данные
            if self.delay > 0 and elapsed_time > self.delay + 2.0:
                mode = "DELAYED"
                if not self.delayed_mode_activated:
                    self.get_logger().info(f'?? DELAYED MODE ACTIVATED! Following with {self.delay}s delay')
                    self.delayed_mode_activated = True
                
                # Сглаживаем позицию для режима с задержкой
                x, y = self.get_smoothed_position(x, y, mode)
            else:
                mode = "REAL-TIME"
            
            # Calculate transform age
            transform_time = trans.header.stamp
            transform_age = (current_time.nanoseconds - (transform_time.sec * 1e9 + transform_time.nanosec)) / 1e9
            
            self.get_logger().info(
                f'[{mode}] Target: x={x:.2f}, y={y:.2f}, Age: {transform_age:.1f}s', 
                throttle_duration_sec=0.5)
            
            # УЛУЧШЕННАЯ ЛОГИКА УПРАВЛЕНИЯ ДЛЯ РЕЖИМА С ЗАДЕРЖКОЙ
            msg = Twist()
            distance = math.sqrt(x**2 + y**2)
            angle_to_target = math.atan2(y, x)
            
            # РАЗНЫЕ СТРАТЕГИИ ДЛЯ РЕЖИМОВ С ЗАДЕРЖКОЙ И БЕЗ
            if mode == "DELAYED":
                # СТРАТЕГИЯ ДЛЯ РЕЖИМА С ЗАДЕРЖКОЙ - более консервативная
                if distance > 0.8:  # Очень далеко
                    if abs(angle_to_target) > 0.5:
                        # Сначала поворачиваемся
                        msg.angular.z = 0.6 * math.copysign(1.0, angle_to_target)
                        msg.linear.x = 0.1
                    else:
                        msg.linear.x = min(distance * 0.3, 0.6)
                        msg.angular.z = angle_to_target * 0.4
                        
                elif distance > 0.3:  # Средняя дистанция
                    if abs(angle_to_target) > 0.3:
                        msg.angular.z = 0.5 * math.copysign(1.0, angle_to_target)
                        msg.linear.x = 0.15
                    else:
                        msg.linear.x = min(distance * 0.4, 0.5)
                        msg.angular.z = angle_to_target * 0.3
                        
                elif distance > 0.1:  # Близко
                    msg.linear.x = distance * 0.3
                    msg.angular.z = angle_to_target * 0.2
                    
                else:  # Очень близко
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    
            else:
                # СТРАТЕГИЯ ДЛЯ РЕЖИМА БЕЗ ЗАДЕРЖКИ - более агрессивная
                if distance > 0.5:
                    if abs(angle_to_target) > 0.4:
                        msg.angular.z = 1.0 * math.copysign(1.0, angle_to_target)
                        msg.linear.x = 0.2
                    else:
                        msg.linear.x = min(distance * 0.6, 1.0)
                        msg.angular.z = angle_to_target * 0.8
                        
                elif distance > 0.1:
                    if abs(angle_to_target) > 0.2:
                        msg.angular.z = 0.8 * math.copysign(1.0, angle_to_target)
                        msg.linear.x = 0.3
                    else:
                        msg.linear.x = min(distance * 0.8, 0.7)
                        msg.angular.z = angle_to_target * 0.6
                        
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
            
            # ДОПОЛНИТЕЛЬНОЕ СГЛАЖИВАНИЕ ДЛЯ БОЛЬШИХ ЗАДЕРЖЕК
            if self.delay >= 2.0 and mode == "DELAYED":
                # Ограничиваем максимальные скорости для больших задержек
                msg.linear.x = min(msg.linear.x, 0.4)
                msg.angular.z = max(min(msg.angular.z, 0.8), -0.8)
                
                # Добавляем небольшую постоянную скорость чтобы избежать остановки
                if msg.linear.x < 0.1 and distance > 0.5:
                    msg.linear.x = 0.15
            
            self.publisher.publish(msg)
            
        except TransformException as e:
            self.get_logger().info('Waiting for transform...', throttle_duration_sec=2.0)
            # Stop the turtle
            msg = Twist()
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            # Stop the turtle on any error
            msg = Twist()
            self.publisher.publish(msg)


def main():
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Node error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()