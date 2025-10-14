#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys
import os

# Добавляем путь к сгенерированным модулям
sys.path.insert(0, '/home/alise/ros2_ws/install/action_cleaning_robot/rosidl_generator_py')

try:
    from action_cleaning_robot.action import CleaningTask
    print("SUCCESS: Imported CleaningTask action in client")
except ImportError as e:
    print(f"ERROR: Could not import CleaningTask in client: {e}")
    sys.exit(1)


class CleaningClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        self._action_client = ActionClient(self, CleaningTask, 'CleaningTask')
        print('Cleaning Action Client started')

    def send_sequence(self):
        """Посылаем последовательность задач"""
        # Ждем сервер
        print('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            print('Action server not available')
            return False
            
        tasks = [
            ("clean_square", 2.0, 0.0, 0.0),
            ("return_home", 0.0, 5.5, 5.5),
        ]
        
        for i, (task_type, area_size, target_x, target_y) in enumerate(tasks):
            print(f'\n{"="*50}')
            print(f'Testing task {i+1}/{len(tasks)}: {task_type}')
            print(f'{"="*50}')
            
            if not self._send_goal(task_type, area_size, target_x, target_y):
                print('Task sequence failed')
                return False
                
        print('\n All tasks completed successfully!')
        return True
        
    def _send_goal(self, task_type, area_size, target_x, target_y):
        """Отправляет цель и ждет результат"""
        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = area_size
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        
        print(f'Sending goal: {task_type}')
        
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback)
        
        future.add_done_callback(self._goal_response_callback)
        
        # Ждем завершения
        self.result_received = False
        self.result_success = False
        
        while not self.result_received:
            rclpy.spin_once(self, timeout_sec=1.0)
            
        return self.result_success
        
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected by server')
            self.result_received = True
            self.result_success = False
            return
            
        print('Goal accepted by server')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
        
    def _result_callback(self, future):
        try:
            result = future.result().result
            print('=' * 50)
            print('FINAL RESULT:')
            print(f'   Success: {result.success}')
            print(f'   Cleaned Points: {result.cleaned_points}')
            print(f'   Total Distance: {result.total_distance:.2f} meters')
            print('=' * 50)
            
            self.result_received = True
            self.result_success = result.success
            
        except Exception as e:
            print(f'Error getting result: {e}')
            self.result_received = True
            self.result_success = False
        
    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f'Progress: {feedback.progress_percent:3d}% | Points: {feedback.current_cleaned_points:2d} | Position: ({feedback.current_x:5.2f}, {feedback.current_y:5.2f})')


def main():
    rclpy.init()
    client = CleaningClient()
    
    # Запускаем тесты через 2 секунды
    client.create_timer(2.0, lambda: client.send_sequence())
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        print('Client interrupted by user')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
