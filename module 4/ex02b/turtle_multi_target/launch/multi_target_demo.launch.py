from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import json


def generate_launch_description():
    
    # Правильный формат JSON для spawn
    turtle2_spawn = json.dumps({"x": 5.0, "y": 5.0, "theta": 0.0, "name": "turtle2"})
    turtle3_spawn = json.dumps({"x": 3.0, "y": 8.0, "theta": 0.0, "name": "turtle3"})
    
    return LaunchDescription([
        # Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        
        # Spawn turtle2 after delay - FIXED JSON FORMAT
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                        turtle2_spawn
                    ],
                    output='screen'
                )
            ]
        ),
        
        # Spawn turtle3 after delay - FIXED JSON FORMAT
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                        turtle3_spawn
                    ],
                    output='screen'
                )
            ]
        ),
        
        # Start TF broadcasters
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='turtle_tf2_broadcaster',
                    name='broadcaster1',
                    parameters=[{'turtlename': 'turtle1'}],
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='turtle_tf2_broadcaster',
                    name='broadcaster2',
                    parameters=[{'turtlename': 'turtle2'}],
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='turtle_tf2_broadcaster',
                    name='broadcaster3',
                    parameters=[{'turtlename': 'turtle3'}],
                    output='screen'
                )
            ]
        ),
        
        # Start target switcher
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='target_switcher',
                    name='target_switcher',
                    parameters=[{'switch_threshold': 1.5}],
                    output='screen'
                )
            ]
        ),
        
        # Start turtle controller
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='turtle_controller',
                    name='turtle_controller',
                    parameters=[{'switch_threshold': 1.5}],
                    output='screen'
                )
            ]
        ),
    ])
