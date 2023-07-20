### Script Creator: Fathima AlAhmed
### Maintainer: Saqeef Tehnan Manna


### This launch file is the one to be used which includes multiple other 
### launch files like spawning of the robot, running navigation, recording data and sending goal.

# July 8th, 2023

import os
from sys import executable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler , SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable
import yaml
import numpy as np

def generate_launch_description():

    params_file = os.environ['PARAMS_FILE']
    specs= os.path.join(
        get_package_share_directory('benchmarking_tool'),
        'config',
        params_file+'.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    controllers_num=robot_specs['controllers_num']
    controller_type=robot_specs['controller_type']
    x = robot_specs['spawn_pose_x']     
    y = robot_specs['spawn_pose_y']        
    yaw = robot_specs['spawn_pose_yaw'] 
    state='"state: {name: turtlebot3_waffle, pose: {position:{x: '+str(x)+', y: '+str(y)+', z: +0.0}, orientation:{x: 0.0, y: 0.0, z: '+str(np.sin(np.deg2rad(90)/2))+' , w: '+str(np.cos(np.deg2rad(90)/2))+' }}, reference_frame: world}"'
    print(state)
    reset=ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                '/set_entity_state ',
                'gazebo_msgs/SetEntityState ',
                '"state: {name: turtlebot3_waffle, pose: {position:{x: '+str(x)+', y: '+str(y)+', z: 0.0}, orientation:{x: 0.0, y: 0.0, z: '+str(np.sin(yaw/2))+' , w: '+str(np.cos(yaw/2))+' }}, reference_frame: world}"'
            ]],
            shell=True
    )  

    ld = LaunchDescription()
    ld.add_action(reset)
    return ld