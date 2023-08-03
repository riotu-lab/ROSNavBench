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


params_file = os.environ['PARAMS_FILE']

def generate_launch_description():

        
    #map path and controller type 
    specs=os.path.join(
        get_package_share_directory('benchmarking_tool'),
        'config',
        params_file+'.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    map_name=robot_specs['map_name']     
    nav_config=robot_specs['nav_config']
    x=robot_specs['spawn_pose_x']     
    y=robot_specs['spawn_pose_y']
    yaw=robot_specs['spawn_pose_yaw']
        
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = LaunchConfiguration(
        'map',
        default = os.path.join(
            get_package_share_directory('benchmarking_tool'),
            'map',
            'map.yaml'))

    # Opening the model template 
    controller_template = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            nav_config+'.yaml')
    with open(controller_template, 'r') as infp:
        template = infp.read()
        
    #Generating a config file for each spawned robot( namely changing the intial pose in rviz)
 
    data = {
            "x_pose":x ,
            "y_pose":y,
            "yaw":yaw ,
           }
        
    j2_template = Template(template)
        
    fileName=nav_config+'.yaml'
    with open(fileName, "w") as f:
        f.write(j2_template.render(data))
    dst=os.path.join(get_package_share_directory('turtlebot3_navigation2'),'param')

    os.rename(os.getcwd()+'/'+fileName, dst+'/'+fileName)

    param_dir = LaunchConfiguration(
        'params_file',
        default = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            nav_config+'.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
            get_package_share_directory('benchmarking_tool'),
            'map',
            map_name),
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
            default_value = '1.0',
            description = 'Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'y_pose',
            default_value = '2.0',
            description = 'Use simulation (Gazebo) clock if true'),            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments = {
                'map': map_dir,
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
