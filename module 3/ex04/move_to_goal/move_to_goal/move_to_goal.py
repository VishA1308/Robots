#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys

class MoveToGoal(Node):
    def __init__(self, target_x, target_y, target_theta):
        super().__init__('move_to_goal')
        
        # Параметры цели
        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = target_theta
        
        # Текущая позиция черепахи
        self.current_pose = None
        
        # Параметры управления
        self.linear_tolerance = 0.1
        self.angular_tolerance = 0.05
        self.max_linear_speed = 2.0
        self.max_angular_speed = 2.0
        self.kp_linear = 1.5
        self.kp_angular = 6.0
        
        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, 
            '/turtle1/pose', 
            self.pose_callback, 
            10
        )
        
        # Таймер для управления
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Moving to goal: x={target_x}, y={target_y}, theta={target_theta}')

    def pose_callback(self, msg):
        """Callback для получения текущей позиции черепахи"""
        self.current_pose = msg

    def calculate_distance(self, x1, y1, x2, y2):
        """Вычисление расстояния между двумя точками"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def normalize_angle(self, angle):
        """Нормализация угла в диапазон [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        """Основной цикл управления"""
        if self.current_pose is None:
            return
            
        current_x = self.current_pose.x
        current_y = self.current_pose.y
        current_theta = self.current_pose.theta
        
        # Вычисление расстояния до цели
        distance_to_goal = self.calculate_distance(
            current_x, current_y, self.target_x, self.target_y
        )
        
        # Вычисление угла до цели
        angle_to_goal = math.atan2(
            self.target_y - current_y, 
            self.target_x - current_x
        )
        
        # Разница углов
        angle_error = self.normalize_angle(angle_to_goal - current_theta)
        
        # Создание сообщения управления
        cmd_vel = Twist()
        
        if distance_to_goal > self.linear_tolerance:
            # Фаза движения к позиции
            if abs(angle_error) > self.angular_tolerance:
                # Поворот в направлении цели
                cmd_vel.angular.z = max(min(
                    self.kp_angular * angle_error, 
                    self.max_angular_speed
                ), -self.max_angular_speed)
            else:
                # Движение вперед с корректировкой направления
                linear_speed = min(
                    self.kp_linear * distance_to_goal, 
                    self.max_linear_speed
                )
                cmd_vel.linear.x = linear_speed
                
                # Небольшая угловая коррекция
                cmd_vel.angular.z = self.kp_angular * angle_error * 0.5
        else:
            # Фаза достижения конечной ориентации
            final_angle_error = self.normalize_angle(self.target_theta - current_theta)
            
            if abs(final_angle_error) > self.angular_tolerance:
                cmd_vel.angular.z = max(min(
                    self.kp_angular * final_angle_error, 
                    self.max_angular_speed
                ), -self.max_angular_speed)
            else:
                # Цель достигнута
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.get_logger().info('Goal reached!')
                self.timer.cancel()  # Остановка таймера
                rclpy.shutdown()
                return
        
        # Публикация команды
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Логирование каждые 2 секунды
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f'Position: ({current_x:.2f}, {current_y:.2f}), '
                f'Target: ({self.target_x:.2f}, {self.target_y:.2f}), '
                f'Distance: {distance_to_goal:.2f}, '
                f'Angle error: {math.degrees(angle_error):.1f}°'
            )

def main(args=None):
    # Проверка аргументов командной строки
    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal <x> <y> <theta>")
        print("Example: ros2 run move_to_goal move_to_goal 5.0 5.0 1.57")
        return
    
    try:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
        target_theta = float(sys.argv[3])
    except ValueError:
        print("Error: All parameters must be numbers")
        return
    
    # Проверка допустимости координат
    if target_x < 0 or target_x > 11 or target_y < 0 or target_y > 11:
        print("Warning: Target coordinates should be within turtlesim bounds (0-11)")
    
    rclpy.init(args=args)
    
    try:
        move_to_goal = MoveToGoal(target_x, target_y, target_theta)
        rclpy.spin(move_to_goal)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
