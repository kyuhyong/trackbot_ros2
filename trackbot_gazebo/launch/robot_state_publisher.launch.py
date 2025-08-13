#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('trackbot_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    xacro_file = os.path.join(
        pkg, 'models', 'trackbot_vehicle', 'model.urdf.xacro'
    )
    # Build URDF via xacro at launch time (each token its own list element)
    robot_description_cmd = Command(['xacro', TextSubstitution(text=' '), xacro_file,  TextSubstitution(text=' '),' use_gazebo:=true'])

    # Force it to be a plain string parameter
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description='TF frame prefix if running multiple robots'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                # Optional: prefix frames when you run multiple robots
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
            }],
        ),
    ])
