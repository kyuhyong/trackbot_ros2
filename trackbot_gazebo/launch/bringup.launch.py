#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_trackbot = get_package_share_directory('trackbot_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World path
    world_path = os.path.join(pkg_trackbot, 'worlds', 'empty_world.sdf')
    model_path = os.path.join(pkg_trackbot, 'models', 'trackbot_vehicle', 'model.sdf')

    # Launch Ignition Gazebo with world
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items()
    )

    # Spawn robot into Gazebo
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'trackbot_vehicle',
            '-file', model_path,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Bridge /cmd_vel from ROS 2 → Gazebo
    bridge_cmd_vel = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    # Bridge /odom and /joint_states back to ROS 2
    bridge_feedback = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        output='screen'
    )

    return LaunchDescription([
        launch_sim,
        spawn_robot,
        bridge_cmd_vel,
        bridge_feedback,
    ])