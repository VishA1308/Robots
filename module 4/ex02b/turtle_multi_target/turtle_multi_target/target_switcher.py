import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String


class TargetSwitcher(Node):
    def __init__(self):
        super().__init__('target_switcher')
        
        self.switch_threshold = self.declare_parameter('switch_threshold', 1.0).get_parameter_value().double_value
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.target_pub = self.create_publisher(String, '/current_target_name', 10)
        
        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.current_target_index = 0
        
        # Подписываемся на команды от клавиатуры
        self.keyboard_sub = self.create_subscription(
            String,
            '/keyboard_switch',
            self.keyboard_switch_callback,
            10
        )
        
        # Подписываемся на автоматические переключения от контроллера
        self.auto_switch_sub = self.create_subscription(
            String,
            '/switch_target',
            self.auto_switch_callback,
            10
        )
        
        self.angle = 0.0
        self.timer = self.create_timer(0.1, self.broadcast_transforms)
        
        # Публикуем начальную цель
        self.publish_current_target()
        
        self.get_logger().info(f'Target switcher started. Initial target: {self.get_current_target()}')
        self.get_logger().info('Switch threshold: %.2f' % self.switch_threshold)

    def get_current_target(self):
        return self.targets[self.current_target_index]

    def publish_current_target(self):
        """Публикация текущей цели"""
        msg = String()
        msg.data = self.get_current_target()
        self.target_pub.publish(msg)

    def next_target(self):
        self.current_target_index = (self.current_target_index + 1) % len(self.targets)
        new_target = self.get_current_target()
        self.get_logger().info(f'Переключение цели: {new_target}')
        
        self.publish_current_target()

    def keyboard_switch_callback(self, msg):
        """Обработчик команд от клавиатуры"""
        if msg.data == 'switch':
            self.next_target()

    def auto_switch_callback(self, msg):
        """Обработчик автоматических переключений от контроллера"""
        if msg.data in self.targets:
            target_index = self.targets.index(msg.data)
            if target_index != self.current_target_index:
                self.current_target_index = target_index
                self.get_logger().info(f'Автоматическое переключение на: {msg.data}')
                self.publish_current_target()

    def broadcast_transforms(self):
        """Публикация трансформаций для всех целей"""
        self.angle += 0.05
        
        # Carrot1 - вращается вокруг turtle1
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'turtle1'
        t1.child_frame_id = 'carrot1'
        
        radius = 2.0
        t1.transform.translation.x = radius * math.cos(self.angle)
        t1.transform.translation.y = radius * math.sin(self.angle)
        t1.transform.translation.z = 0.0
        
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t1)
        
        # Carrot2 - вращается вокруг turtle3
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'turtle3'
        t2.child_frame_id = 'carrot2'
        
        radius2 = 1.5
        t2.transform.translation.x = radius2 * math.cos(-self.angle * 1.5)
        t2.transform.translation.y = radius2 * math.sin(-self.angle * 1.5)
        t2.transform.translation.z = 0.0
        
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t2)
        
        # Static target - фиксированная позиция
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'world'
        t3.child_frame_id = 'static_target'
        
        t3.transform.translation.x = 8.0
        t3.transform.translation.y = 2.0
        t3.transform.translation.z = 0.0
        
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t3)


def main():
    rclpy.init()
    node = TargetSwitcher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
