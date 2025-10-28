from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.conditions import IfCondition
import launch


def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'radius',
            default_value='2.0',
            description='Radius of carrot rotation around turtle1'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation',
            default_value='1',
            description='Direction of rotation (1 for clockwise, -1 for counter-clockwise)'
        ),
        
        # Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        
        # Spawn second turtle after delay
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                         '{x: 5.0, y: 5.0, theta: 0.0, name: "turtle2"}'],
                    output='screen'
                )
            ]
        ),
        
        # Start broadcaster for turtle1
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='carrot_follower',
                    executable='turtle_tf2_broadcaster',
                    name='broadcaster1',
                    parameters=[{'turtlename': 'turtle1'}],
                    output='screen'
                )
            ]
        ),
        
        # Start broadcaster for turtle2
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='carrot_follower',
                    executable='turtle_tf2_broadcaster',
                    name='broadcaster2',
                    parameters=[{'turtlename': 'turtle2'}],
                    output='screen'
                )
            ]
        ),
        
        # Start carrot broadcaster
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='carrot_follower',
                    executable='carrot_broadcaster',
                    name='carrot_broadcaster',
                    parameters=[
                        {'radius': LaunchConfiguration('radius')},
                        {'direction_of_rotation': LaunchConfiguration('direction_of_rotation')}
                    ],
                    output='screen'
                )
            ]
        ),
        
        # Start listener last
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='carrot_follower',
                    executable='turtle_tf2_listener',
                    name='listener',
                    parameters=[
                        {'target_frame': 'carrot'},
                        {'turtlename': 'turtle2'}
                    ],
                    output='screen'
                )
            ]
        ),
    ])
