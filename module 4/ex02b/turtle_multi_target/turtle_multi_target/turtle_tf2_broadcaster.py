import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose


class TurtleTF2Broadcaster(Node):
    def __init__(self):
        super().__init__('turtle_tf2_broadcaster')
        
        self.turtlename = self.declare_parameter('turtlename', 'turtle1').get_parameter_value().string_value
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            10)
        
        self.get_logger().info(f'Broadcaster started for {self.turtlename}')

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename
        
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(msg.theta / 2)
        qw = math.cos(msg.theta / 2)
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TurtleTF2Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
