import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose


class Broadcaster(Node):

    def __init__(self):
        super().__init__('broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to turtle poses
        self.turtle1_pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1_pose_callback,
            10)
        self.turtle2_pose_subscription = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.turtle2_pose_callback,
            10)
        
        self.get_logger().info('Broadcaster started')

    def turtle1_pose_callback(self, msg):
        self.send_transform(msg, 'turtle1')

    def turtle2_pose_callback(self, msg):
        self.send_transform(msg, 'turtle2')

    def send_transform(self, pose, turtle_name):
        t = TransformStamped()
        
        # Set the frame and child frame
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = turtle_name
        
        # Set the translation
        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(pose.theta / 2.0)
        t.transform.rotation.w = math.cos(pose.theta / 2.0)
        
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()