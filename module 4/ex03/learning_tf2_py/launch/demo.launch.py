from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delay',
            default_value='5.0',
            description='Time delay in seconds'
        ),
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        Node(
            package='learning_tf2_py',
            executable='broadcaster',
            name='broadcaster'
        ),
        
        Node(
            package='learning_tf2_py',
            executable='listener',
            name='listener',
            parameters=[{'delay': LaunchConfiguration('delay')}]
        ),
    ])