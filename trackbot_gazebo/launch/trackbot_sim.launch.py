from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sdf_xacro_path = os.path.join(
        get_package_share_directory('trackbot_gazebo'),
        'models',
        'trackbot_vehicle',
        'model.sdf.xacro'
    )
    sdf_output_path = os.path.join(
        get_package_share_directory('trackbot_gazebo'),
        'models',
        'trackbot_vehicle',
        'trackbot_vehicle.sdf'
    )

    # Generate SDF from xacro
    generate_sdf_cmd = ExecuteProcess(
        cmd=['xacro', sdf_xacro_path, '-o', sdf_output_path],
        output='screen'
    )

    # Spawn the robot in simulation (using ros_gz_sim or gazebo_ros)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', sdf_output_path, '-name', 'trackbot_vehicle'],
        output='screen'
    )

    return LaunchDescription([
        generate_sdf_cmd,
        spawn_robot
    ])