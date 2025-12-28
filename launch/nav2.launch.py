# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

# Multiple edits have been made thoughout the document by Fatimah Alhamed. 
import os
from jinja2 import Template
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
from ROSNavBench.path_utils import resolve_paths_in_config

def generate_launch_description():
    # Get the name of config file of the current experiment
    # Read from environment variable (set by main.launch.py)
    specs = os.environ.get('PARAMS_FILE')
    if not specs:
        raise ValueError("PARAMS_FILE environment variable must be set")

    # Opening the config file to take the experiment data such as the path of the navigation configuration
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
    
    # Resolve all paths in the config relative to the config file location
    resolve_paths_in_config(robot_specs, specs)

    map_path = robot_specs['map_path']
    nav_config = robot_specs['nav_config']
    
 
    if robot_specs['trajectory_type'] == 'user_defined':
        x=robot_specs['user_defined_trajectories'][0]["spawn_pose"]["x"]
        y=robot_specs['user_defined_trajectories'][0]["spawn_pose"]["y"]    
        yaw=robot_specs['user_defined_trajectories'][0]["spawn_pose"]["yaw"]
    elif robot_specs['trajectory_type'] == 'auto_generated':
        x=robot_specs['auto_generated_trajectory']["spawn_pose"]["x"]
        y=robot_specs['auto_generated_trajectory']["spawn_pose"]["y"]    
        yaw=robot_specs['auto_generated_trajectory']["spawn_pose"]["yaw"]    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Seting the path of the map
    map_dir = LaunchConfiguration(
        'map',
        default = os.path.join(
            get_package_share_directory('ROSNavBench'),
            'maps',
            'warehouse_slam_toolbox.yaml'))
    # Seting the path of the navigation configuration file
    nav2_param_file = LaunchConfiguration(
        'nav2_params_file',
        default=nav_config)

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('ROSNavBench'),
        'config',
        'rviz_config.rviz')

    return LaunchDescription([ 
        DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description = 'Full path to map file to load'),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=nav_config,
            description='Full path to Nav2 params file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'true',
            description = 'Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'x_pose',
            default_value = str(x),
            description = 'Initial x pose'),
        DeclareLaunchArgument(
            'y_pose',
            default_value = str(y),
            description = 'Initial y pose'),
        DeclareLaunchArgument(
            'yaw_pose',
            default_value = str(yaw),
            description = 'Initial yaw pose'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments = {
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_file,
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'yaw_pose': LaunchConfiguration('yaw_pose')}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
            
        # Static transform publisher for map->odom frame
        # This ensures the map frame exists until AMCL takes over and publishes the transform
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map_to_odom_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
