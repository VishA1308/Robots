#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

from action_cleaning_robot.action import CleaningTask


class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')
        
        # Константы
        self.MOVEMENT_SPEED = 1.0
        self.ROTATION_SPEED = 2.0
        self.CLEANING_RESOLUTION = 0.03
        self.POSITION_TOLERANCE = 0.015
        self.ANGLE_TOLERANCE = 0.01
        
        # Инициализация
        self.current_pose = None
        self.velocity_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_update, 10)
        
        # Action Server
        self._action_server = ActionServer(
            self, CleaningTask, 'CleaningTask', self.processing_callback
        )
        
        self.get_logger().info('Cleaning Action Server initialized and running')

    def pose_update(self, msg):
        """Обновление текущей позиции черепахи"""
        self.current_pose = msg

    def wait_for_pose(self):
        """Ожидание получения первой позиции"""
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def normalize_angle(self, angle):
        """Нормализация угла в диапазон [-π, π]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def rotate_to_angle(self, target_angle):
        """Поворот к заданному углу"""
        velocity_cmd = Twist()
        
        while rclpy.ok():
            angle_diff = self.normalize_angle(target_angle - self.current_pose.theta)
            
            if abs(angle_diff) > self.ANGLE_TOLERANCE:
                velocity_cmd.linear.x = 0.0
                velocity_cmd.angular.z = self.ROTATION_SPEED * angle_diff
                self.velocity_publisher.publish(velocity_cmd)
            else:
                velocity_cmd.angular.z = 0.0
                self.velocity_publisher.publish(velocity_cmd)
                break
                
            rclpy.spin_once(self, timeout_sec=0.01)

    def move_to_position(self, target_x, target_y):
        """Движение к заданной позиции"""
        velocity_cmd = Twist()
        
        # Поворот в направлении цели
        target_angle = math.atan2(target_y - self.current_pose.y, 
                                 target_x - self.current_pose.x)
        self.rotate_to_angle(target_angle)
        
        # Движение к цели
        while rclpy.ok():
            distance = math.hypot(target_x - self.current_pose.x, 
                                target_y - self.current_pose.y)
            
            if distance > self.POSITION_TOLERANCE:
                # Коррекция направления во время движения
                current_target_angle = math.atan2(target_y - self.current_pose.y,
                                                target_x - self.current_pose.x)
                angle_error = self.normalize_angle(current_target_angle - self.current_pose.theta)
                
                velocity_cmd.linear.x = self.MOVEMENT_SPEED
                velocity_cmd.angular.z = self.ROTATION_SPEED * angle_error
                self.velocity_publisher.publish(velocity_cmd)
            else:
                velocity_cmd.linear.x = 0.0
                velocity_cmd.angular.z = 0.0
                self.velocity_publisher.publish(velocity_cmd)
                break
                
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop_movement(self):
        """Остановка движения"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.velocity_publisher.publish(stop_cmd)

    def clean_square_area(self, goal_handle, start_x, start_y, area_size):
        """Очистка квадратной области"""
        if area_size <= 0.1:
            self.get_logger().error('Invalid area size: too small')
            return False, 0

        cleaning_lines = int(area_size / self.CLEANING_RESOLUTION)
        cleaned_points = 0

        for line in range(cleaning_lines + 1):
            current_y = start_y + line * self.CLEANING_RESOLUTION
            
            if current_y > start_y + area_size:
                break

            # Движение вперед или назад в зависимости от четности линии
            if line % 2 == 0:
                self.move_to_position(start_x + area_size, current_y)
            else:
                self.move_to_position(start_x, current_y)
                
            cleaned_points += 1
            
            # Отправка прогресса
            progress = int((line + 1) / (cleaning_lines + 1) * 100)
            goal_handle.publish_feedback(CleaningTask.Feedback(
                progress_percent=progress,
                current_cleaned_points=cleaned_points,
                current_x=self.current_pose.x,
                current_y=self.current_pose.y
            ))

        # Возврат в начальную позицию
        self.move_to_position(start_x, start_y)
        self.rotate_to_angle(self.current_pose.theta)  # Сохранение ориентации
        
        return True, cleaned_points

    def return_to_home(self, goal_handle, home_x, home_y):
        """Возврат в домашнюю позицию"""
        start_angle = self.current_pose.theta
        
        self.move_to_position(home_x, home_y)
        self.rotate_to_angle(start_angle)
        
        # Отправка финального feedback
        goal_handle.publish_feedback(CleaningTask.Feedback(
            progress_percent=100,
            current_cleaned_points=0,
            current_x=self.current_pose.x,
            current_y=self.current_pose.y
        ))
        
        distance = math.hypot(self.current_pose.x - home_x, 
                            self.current_pose.y - home_y)
        return True, distance

    def processing_callback(self, goal_handle):
        """Основной обработчик action goals"""
        self.get_logger().info(f'Received goal: {goal_handle.request.task_type}')
        
        # Ожидание получения позиции
        self.wait_for_pose()
        
        # Сохранение начального состояния
        start_x, start_y = self.current_pose.x, self.current_pose.y
        start_angle = self.current_pose.theta
        
        result = CleaningTask.Result()
        task_type = goal_handle.request.task_type
        
        try:
            if task_type == "clean_square":
                success, points = self.clean_square_area(
                    goal_handle, start_x, start_y, goal_handle.request.area_size
                )
                result.success = success
                result.cleaned_points = points
                result.total_distance = goal_handle.request.area_size * points
                
            elif task_type == "return_home":
                success, distance = self.return_to_home(
                    goal_handle, goal_handle.request.target_x, goal_handle.request.target_y
                )
                result.success = success
                result.cleaned_points = 0
                result.total_distance = distance
                
            else:
                self.get_logger().error(f'Unknown task type: {task_type}')
                result.success = False
                goal_handle.abort()
                return result

            # Завершение задачи
            if result.success:
                self.get_logger().info('Task completed successfully')
                goal_handle.succeed()
            else:
                self.get_logger().error('Task failed')
                goal_handle.abort()
                
        except Exception as e:
            self.get_logger().error(f'Error during task execution: {str(e)}')
            result.success = False
            goal_handle.abort()
            
        finally:
            self.stop_movement()
            
        return result


def main(args=None):
    rclpy.init(args=args)
    
    try:
        action_server = CleaningActionServer()
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
