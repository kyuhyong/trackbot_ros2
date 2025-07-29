import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate xacro file
    xacro_model = os.path.join(
        get_package_share_directory('trackbot_gazebo'),
        'models',
        'trackbot_vehicle',
        'model.urdf.xacro'
    )

    # Temporary output SDF path
    generated_sdf = os.path.join(
        get_package_share_directory('trackbot_gazebo'),
        'models',
        'trackbot_vehicle',
        'model.sdf'
    )
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory('trackbot_gazebo'), 'models')
    )

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='0.0', description='X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.3', description='Z (height) position'),

        ExecuteProcess(
            cmd=['xacro', xacro_model, '-o', generated_sdf],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create', 
                '-file', generated_sdf, 
                '-name', 'trackbot_vehicle',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose
            ],
            output='screen'
        )
    ])
