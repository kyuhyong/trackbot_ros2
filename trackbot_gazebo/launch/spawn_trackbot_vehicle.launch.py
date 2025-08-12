import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg = get_package_share_directory('trackbot_gazebo')
    
    generated_urdf = os.path.join(pkg, 'models', 'trackbot_vehicle', 'model.urdf')
    generated_sdf  = os.path.join(pkg, 'models', 'trackbot_vehicle', 'model.sdf')
    
    xacro_model = os.path.join(pkg, 'models', 'trackbot_vehicle', 'model.urdf.xacro')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.3')

    # 1) Build the robot description string with Gazebo bits enabled
    robot_urdf = Command(['xacro ', xacro_model, ' use_gazebo:=true'])

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg, 'models')
        ),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.3', description='Z (height) position'),

        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create', 
                '-string', robot_urdf,
                '-name', 'trackbot_vehicle',
                '-x', x_pose, '-y', y_pose, '-z', z_pose
            ],
            output='screen'
        )
    ])
