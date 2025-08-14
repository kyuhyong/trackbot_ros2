#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('trackbot_controller')
    default_cfg = os.path.join(pkg, 'config', 'teleop_keyboard.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_cfg,
            description='YAML with max speeds & step sizes'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='Twist output topic'
        ),
        Node(
            package='trackbot_controller',
            executable='teleop_keyboard',          # your custom node
            name='trackbot_teleop_keyboard',
            output='screen',
            emulate_tty=True,                      # keeps stdout tidy; does not create a new TTY
            parameters=[
                LaunchConfiguration('config'),
                {'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')}
            ],
        ),
    ])
