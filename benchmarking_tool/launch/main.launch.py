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
        
    controller_type=robot_specs['controller_type']
    x = robot_specs['spawn_pose_x']     
    y = robot_specs['spawn_pose_y']        
    yaw = robot_specs['spawn_pose_yaw']    

    spawn_robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
         FindPackageShare("benchmarking_tool"), '/launch', '/spawn_turtlebot3.launch.py'])
        
            )
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("benchmarking_tool"), '/launch', '/nav2.launch.py'])
 
            )

    pdf_generator=Node(
        name='PDF_generator',
        executable='pdf_generator',
        package='benchmarking_tool',
    )
    ld = LaunchDescription()
    ld.add_action(spawn_robot)
    ld.add_action(nav2)
    ld.add_action(SetEnvironmentVariable(name='controller',value=controller_type[0]))
    nodes=[]
    for j in range(len(controller_type)):
       nodes.append(Node(
        name='follow_path_0',
        executable='follow_path',
        package='benchmarking_tool',
    )) 
    state_nodes=[]
    
    for k in range(len(controller_type)-1):  
        state_nodes.append(ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                '/set_entity_state ',
                'gazebo_msgs/SetEntityState ',
                '"state: {name: turtlebot3, pose: {position:{x: '+str(x)+', y: '+str(y)+', z: 0.0}, orientation:{x: 0.0, y: 0.0, z: '+str(np.sin(yaw/2))+' , w: '+str(np.cos(yaw/2))+' }}, reference_frame: world}"'
            ]],
            shell=True
    ) ) 
    ld.add_action(nodes[0])   
    
    for i in range(len(controller_type)-1):

        ld.add_action(RegisterEventHandler(OnProcessExit(target_action= nodes[i], on_exit=[state_nodes[i]]))) 
        ld.add_action(RegisterEventHandler(OnProcessExit(target_action=state_nodes[i], on_exit=[SetEnvironmentVariable(name='controller',value=controller_type[i+1]), nodes[i+1]])))  
    
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=nodes[len(controller_type)-1], on_exit=[pdf_generator] )))

    return ld
    
