#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _build_actions(context):
    # Resolve paths
    pkg = get_package_share_directory('trackbot_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Resolve launch args (strings here)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    config_path  = LaunchConfiguration('config').perform(context)
    profile      = LaunchConfiguration('profile').perform(context)
    world_override = LaunchConfiguration('world').perform(context)  # optional path
    verbose_str  = LaunchConfiguration('verbose').perform(context)

    # CLI spawn overrides (empty string means "no override")
    x_pose_cli = LaunchConfiguration('x_pose').perform(context)
    y_pose_cli = LaunchConfiguration('y_pose').perform(context)
    z_pose_cli = LaunchConfiguration('z_pose').perform(context)

    # --- Load YAML config ---
    try:
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
    except Exception as e:
        raise RuntimeError(f"Failed to read YAML config at '{config_path}': {e}")

    # Pick world entry
    worlds = cfg.get('worlds', {})
    selected = worlds.get(profile, {})
    if not selected and not world_override:
        # allow top-level default pointer
        default_key = cfg.get('default')
        if default_key and default_key in worlds:
            profile = default_key
            selected = worlds.get(default_key, {})
        else:
            raise RuntimeError(
                f"Profile '{profile}' not found in {config_path} and no 'world' override provided."
            )

    # Determine world path
    if world_override:
        world_path = world_override
    else:
        rel = selected.get('file', '')
        world_path = rel if os.path.isabs(rel) else os.path.join(pkg, rel)

    if not os.path.exists(world_path):
        raise FileNotFoundError(f"SDF world not found: {world_path}")

    # Per-world options
    paused = bool(selected.get('paused', False))
    gui    = bool(selected.get('gui', True))
    extra_args = str(selected.get('extra_args', '')).strip()

    # Spawn defaults from YAML (can be overridden by CLI)
    spawn_cfg = selected.get('spawn', {})
    x_yaml = spawn_cfg.get('x', 0.0)
    y_yaml = spawn_cfg.get('y', 0.0)
    z_yaml = spawn_cfg.get('z', 1.0)

    # Merge precedence: CLI (if provided) > YAML > built-in defaults
    def pick(cli_val, yaml_val, default_val):
        # accept numbers or strings; empty string means "no CLI override"
        if isinstance(cli_val, str) and cli_val.strip() != '':
            return float(cli_val)
        return float(yaml_val if yaml_val is not None else default_val)

    x_pose = pick(x_pose_cli, x_yaml, 0.0)
    y_pose = pick(y_pose_cli, y_yaml, 0.0)
    z_pose = pick(z_pose_cli, z_yaml, 1.0)

    # Verbosity
    vflag = f"-v{int(verbose_str)}" if verbose_str.isdigit() else "-v4"

    # --- Build gz_sim args ---
    run_flag = "" if paused else "-r"
    gzserver_args = f"{run_flag} -s {vflag} {extra_args} {world_path}".strip()
    gzclient_args = f"-g {vflag}"

    actions = []

    actions.append(LogInfo(
        msg=f"[gazebo_world] profile='{profile}', world='{world_path}', gui={gui}, paused={paused}, "
            f"spawn=({x_pose:.3f}, {y_pose:.3f}, {z_pose:.3f})"
    ))

    # --- Resources: models/worlds in package + Fuel cache + existing path ---
    models_dir = os.path.join(pkg, 'models')
    worlds_dir = os.path.join(pkg, 'worlds')
    fuel_cache = os.path.expanduser('~/.ignition/fuel')
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = os.pathsep.join([p for p in [models_dir, worlds_dir, fuel_cache, existing] if p])
    actions.append(SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path))

    # --- Server ---
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': gzserver_args,
                'on_exit_shutdown': 'true'
            }.items()
        )
    )

    # --- Client (optional GUI) ---
    if gui:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
                launch_arguments={'gz_args': gzclient_args}.items()
            )
        )

    # --- Robot State Publisher ---
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    )

    # --- Spawn robot (URDF/xacro -> entity) ---
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'spawn_trackbot_vehicle.launch.py')),
            launch_arguments={
                'x_pose': str(x_pose),
                'y_pose': str(y_pose),
                'z_pose': str(z_pose),
            }.items()
        )
    )

    # --- Bridges (adjust topics for your robot as needed) ---
    actions.append(
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU'
            ],
            output='screen'
        )
    )
    actions.append(
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        )
    )

    return actions


def generate_launch_description():
    pkg = get_package_share_directory('trackbot_gazebo')

    return LaunchDescription([
        # Core config
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(pkg, 'config', 'worlds.yaml'),
            description='YAML file describing available worlds and options.'
        ),
        DeclareLaunchArgument(
            'profile',
            default_value='empty',
            description='Key under worlds: in YAML to select a world configuration.'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Optional absolute/relative SDF path. Overrides YAML "file".'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='4', description='Gazebo verbosity (0–6).'),

        # Spawn overrides (leave empty to use YAML values)
        DeclareLaunchArgument('x_pose', default_value='', description='Override spawn X (meters)'),
        DeclareLaunchArgument('y_pose', default_value='', description='Override spawn Y (meters)'),
        DeclareLaunchArgument('z_pose', default_value='', description='Override spawn Z (meters)'),

        # Build the rest after resolving args & loading YAML
        OpaqueFunction(function=_build_actions),
    ])
