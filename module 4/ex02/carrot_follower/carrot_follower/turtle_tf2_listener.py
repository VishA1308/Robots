import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'carrot').get_parameter_value().string_value

        self.turtlename = self.declare_parameter(
          'turtlename', 'turtle2').get_parameter_value().string_value

        # Create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publisher for turtle2
        self.publisher = self.create_publisher(Twist, f'/{self.turtlename}/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)

        # Counter for failed transformations
        self.failed_attempts = 0
        self.max_failed_attempts = 10

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = self.turtlename

        try:
            # Lookup transformation with timeout
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            
            # Reset failed attempts counter on successful transform
            self.failed_attempts = 0

        except TransformException as ex:
            self.failed_attempts += 1
            if self.failed_attempts <= self.max_failed_attempts:
                self.get_logger().info(
                    f'Waiting for transform {to_frame_rel} to {from_frame_rel}: {ex}')
            elif self.failed_attempts == self.max_failed_attempts + 1:
                self.get_logger().warn(
                    f'Still waiting for transform {to_frame_rel} to {from_frame_rel}. Suppressing further warnings.')
            return

        msg = Twist()
        scale_rotation_rate = 1.0
        scale_forward_speed = 0.5

        msg.angular.z = scale_rotation_rate * math.atan2(
            t.transform.translation.y,
            t.transform.translation.x)

        msg.linear.x = scale_forward_speed * math.sqrt(
            t.transform.translation.x ** 2 +
            t.transform.translation.y ** 2)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
