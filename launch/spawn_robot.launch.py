
import os
import yaml
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ROSNavBench.path_utils import resolve_paths_in_config

def generate_launch_description():
    # Package Paths
    pkg_ros_nav_bench = FindPackageShare('ROSNavBench')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')
    
    # We ignore the original URDF path from config if we are effectively overriding it 
    # with our migrated harmonic URDF, but we keep the plumbing consistent if possible.
    # Ideally we load the URDF path from the environment or argument. 
    # For this migration, we hardcode to the local package SDF/URDF if not provided.
    urdf_file = LaunchConfiguration('urdf_file', default=PathJoinSubstitution(
        [pkg_ros_nav_bench, 'simulations', 'urdf', 'turtlebot3_waffle_pi.urdf']
    ))
    
    # Resource Path for Gazebo to find meshes
    # We assume usage of standard 'turtlebot3_description' which should be in package path.
    # But if we need to add local paths:
    # Resource Path for Gazebo to find meshes
    # Use AppendEnvironmentVariable which is safer and handles existing variables
    gz_resource_path_local = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([pkg_ros_nav_bench, 'simulations', 'models'])
    )

    gz_resource_path_system = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/opt/ros/jazzy/share/turtlebot3_gazebo/models'
    )

    # Add any custom model paths from the experiment config (supports ':' separated lists)
    extra_model_paths = []
    specs = os.environ.get('PARAMS_FILE')
    if specs:
        try:
            with open(specs, 'r') as file:
                robot_specs = yaml.safe_load(file)
            resolve_paths_in_config(robot_specs, specs)
            models_path = robot_specs.get('models_path', '')
            if models_path:
                extra_model_paths = [p for p in models_path.split(':') if p]
        except Exception:
            extra_model_paths = []
    extra_model_path_actions = [
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=path)
        for path in extra_model_paths
    ]

    # Gazebo Sim
    world_file = LaunchConfiguration('world', default=PathJoinSubstitution([pkg_ros_nav_bench, 'simulations', 'worlds', 'turtlebot3.sdf']))

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    # Spawn (Create)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_waffle_pi',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
        ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # TF Bridge (Optional if using robot_state_publisher with GZ plugin for TF)
    # If using gz-sim-diff-drive-system with <publish_odom_tf>true</publish_odom_tf>, GZ publishes /tf.
    # We bridge it above.

    return LaunchDescription([
        *extra_model_path_actions,
        gz_resource_path_local,
        gz_resource_path_system,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('urdf_file', default_value=PathJoinSubstitution(
            [pkg_ros_nav_bench, 'simulations', 'urdf', 'turtlebot3_waffle_pi.urdf']
        )),
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge
    ])
