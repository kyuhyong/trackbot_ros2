#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg = get_package_share_directory('trackbot_gazebo')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.3')

    xacro_file = os.path.join(pkg, 'models', 'trackbot_vehicle', 'model.urdf.xacro')
    
    # Build URDF string at launch time (each token isolated)
    robot_urdf_cmd = Command(['xacro',TextSubstitution(text=' '), xacro_file, TextSubstitution(text=' '), 'use_gazebo:=true'])

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg, 'models')
        ),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.3', description='Z (height) position'),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-string', robot_urdf_cmd,
                '-name', 'trackbot_vehicle',
                '-x', x_pose, '-y', y_pose, '-z', z_pose
            ],
            output='screen'
        )
    ])
