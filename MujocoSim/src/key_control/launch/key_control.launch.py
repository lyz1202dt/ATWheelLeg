from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='key_control',
            executable='key_control_node',
            name='key_control_node',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'linear_step': 0.1,
                'angular_step': 0.1,
                'publish_rate': 50.0,
            }],
        ),
    ])
