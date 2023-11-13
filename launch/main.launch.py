### Script Creator: Fathima AlAhmed
### Maintainer: Saqeef Tehnan Manna


### This launch file is the one to be used which includes multiple other 
### launch files like spawning of the robot, running navigation, sending goal,recording data, and generating report.

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
    # Get the name of config file of the current experiment
    specs = os.environ['PARAMS_FILE']
    # Opening the config file to take the experiment data such as spawn pose
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    controller_type=robot_specs['controller_type']
    planner_type=robot_specs['planner_type']
    x = robot_specs['spawn_pose_x']     
    y = robot_specs['spawn_pose_y']        
    yaw = robot_specs['spawn_pose_yaw']  
    trajectory_type = robot_specs['trajectory_type']
    instances_num= robot_specs['instances_num']
   
    # Node for generating pdf
    pdf_generator=Node(
            name='main_pdf_generator',
            executable='main_pdf_generator',
            package='ROSNavBench',
        )  
     
    if trajectory_type=='circle':
        r= robot_specs['radius']
        x=x+r
        yaw=1.5707963 # 90 degrees  
    # Include launch file for spawning the robot
    spawn_robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
         FindPackageShare("ROSNavBench"), '/launch', '/spawn_robot.launch.py'])
        
            )
    # Include launch file for launching navigation
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("ROSNavBench"), '/launch', '/nav2.launch.py'])
 
            )      
 
   
    ld = LaunchDescription()
    ld.add_action(spawn_robot)
    ld.add_action(nav2)
    ld.add_action(SetEnvironmentVariable(name='planner',value=planner_type[0]))
    ld.add_action(SetEnvironmentVariable(name='controller',value=controller_type[0]))
    ld.add_action(SetEnvironmentVariable(name='round_num',value="1"))
    # Generating  different nodes to publish the type of running controller 
    controller_node=[]
    for j in range(len(controller_type)*len(planner_type)*instances_num):
       controller_node.append(Node(
        name='marker_publisher',
        executable='marker_publisher',
        package='ROSNavBench',
    ))           
    # Generating different nodes for sending the goal and recording the data
    nodes=[]
    for j in range(len(controller_type)*len(planner_type)*instances_num):
       nodes.append(Node(
        name='follow_path_0',
        executable='follow_path',
        package='ROSNavBench',
    )) 
    # Generating different nodes for reseting the pose of the robot to intial pose after each controller scenario   
    state_nodes=[]
    for k in range(len(controller_type)*len(planner_type)*instances_num-1):  
        state_nodes.append(ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                '/set_entity_state ',
                'gazebo_msgs/SetEntityState ',
                '"state: {name: robot, pose: {position:{x: '+str(x)+', y: '+str(y)+', z: 0.0}, orientation:{x: 0.0, y: 0.0, z: '+str(np.sin(yaw/2))+' , w: '+str(np.cos(yaw/2))+' }}, reference_frame: world}"'
            ]],
            shell=True
    ) ) 
        

# Each event will start once  the previous event is done
    for k in range(len(planner_type)):
        for i in range(len(controller_type)):
            for q in range(instances_num):
                if i==0 and k==0 and q==0:
                    ld.add_action(nodes[0])  
                    ld.add_action(controller_node[0]) 
                else:
                    increament=k*len(controller_type)*instances_num+i*instances_num+q-1
                    ld.add_action(RegisterEventHandler(OnProcessExit(target_action= nodes[increament], on_exit=[state_nodes[increament]]))) #edit 
                    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=state_nodes[increament], on_exit=[SetEnvironmentVariable(name='planner',value=planner_type[k]),SetEnvironmentVariable(name='controller',value=controller_type[i]),SetEnvironmentVariable(name='round_num',value=str(increament+2)),nodes[increament+1],controller_node[increament+1]])))  
    
    # Once all events are done, the node of generating a pdf will start
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=nodes[len(controller_type)*len(planner_type)*instances_num-1], on_exit=[pdf_generator] )))

    return ld
    
