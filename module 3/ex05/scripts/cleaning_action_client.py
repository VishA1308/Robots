#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys

from action_cleaning_robot.action import CleaningTask


class CleaningActionClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        self._client = ActionClient(self, CleaningTask, 'CleaningTask')
        self.goal_handle = None

    def wait_for_server(self, timeout_sec=10.0):
        """Ожидание доступности сервера"""
        self.get_logger().info('Waiting for action server...')
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, task_type, area_size=0.0, target_x=0.0, target_y=0.0):
        """Отправка цели на выполнение"""
        if not self.wait_for_server():
            self.get_logger().error('Action server not available')
            return False

        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = area_size
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y

        self.get_logger().info(f'Sending goal: {task_type}')
        
        self._send_goal_future = self._client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_handler
        )
        self._send_goal_future.add_done_callback(self.response_handler)
        
        return True

    def response_handler(self, future):
        """Обработка ответа на цель"""
        try:
            self.goal_handle = future.result()
            
            if not self.goal_handle.accepted:
                self.get_logger().error('Goal rejected by server')
                rclpy.shutdown()
                return

            self.get_logger().info('Goal accepted')
            
            self._result_future = self.goal_handle.get_result_async()
            self._result_future.add_done_callback(self.result_handler)
            
        except Exception as e:
            self.get_logger().error(f'Error in response handler: {str(e)}')

    def feedback_handler(self, feedback_msg):
        """Обработка feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {feedback.progress_percent}% | '
            f'Cleaned: {feedback.current_cleaned_points} | '
            f'Position: ({feedback.current_x:.2f}, {feedback.current_y:.2f})'
        )

    def result_handler(self, future):
        """Обработка результата"""
        try:
            result = future.result().result
            status = 'SUCCESS' if result.success else 'FAILED'
            
            self.get_logger().info(
                f'{status} | '
                f'Points cleaned: {result.cleaned_points} | '
                f'Total distance: {result.total_distance:.2f}m'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error getting result: {str(e)}')
        finally:
            rclpy.shutdown()

    def cancel_goal(self):
        """Отмена текущей цели"""
        if self.goal_handle:
            future = self.goal_handle.cancel_goal_async()
            self.get_logger().info('Goal cancellation requested')


def main(args=None):
    rclpy.init(args=args)
    
    client = CleaningActionClient()
    
    # Параметры по умолчанию
    task_type = 'clean_square'
    area_size = 1.0
    
    # Обработка аргументов командной строки
    if len(sys.argv) > 1:
        task_type = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            area_size = float(sys.argv[2])
        except ValueError:
            client.get_logger().error('Invalid area size')
            return
    
    # Отправка цели
    if task_type == 'clean_square':
        success = client.send_goal(task_type, area_size=area_size)
    elif task_type == 'return_home':
        success = client.send_goal(task_type, target_x=5.5, target_y=5.5)
    else:
        client.get_logger().error(f'Unknown task type: {task_type}')
        return
    
    if success:
        try:
            rclpy.spin(client)
        except KeyboardInterrupt:
            client.get_logger().info('Interrupted by user')
            client.cancel_goal()
    else:
        client.get_logger().error('Failed to send goal')
    
    client.destroy_node()


if __name__ == '__main__':
    main()
