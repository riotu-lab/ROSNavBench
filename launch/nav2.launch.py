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

# Get the name of config file of the current experiment
specs = os.environ['PARAMS_FILE']

def generate_launch_description():

    # Opening the config file to take the experiment data such as the path of the navigation configuration

    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)

    map_path=robot_specs['map_path']
    nav_config=robot_specs['nav_config']
    x=robot_specs['spawn_pose_x']
    y=robot_specs['spawn_pose_y']
    yaw=robot_specs['spawn_pose_yaw']
    trajectory_type= robot_specs['trajectory_type']
    
    if trajectory_type=='circle':
        r= robot_specs['radius']
        x=x+r
        yaw=1.5707963 # 90 degrees
            
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Seting the path of the map
    map_dir = LaunchConfiguration(
        'map',
        default = os.path.join(
            get_package_share_directory('ROSNavBench'),
            'maps',
            'warehouse_slam_toolbox.yaml'))
    # Seting the path of the navigation configuration file
    param_dir = LaunchConfiguration(
        'params_file',
        default =nav_config)

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
            'params_file',
            default_value = param_dir,
            description = 'Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'true',
            description = 'Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'x_pose',
            default_value = str(x),
            description = 'Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'y_pose',
            default_value = str(y),
            description = 'Use simulation (Gazebo) clock if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments = {
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
