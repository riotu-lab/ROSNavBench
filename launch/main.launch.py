
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
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import FindExecutable
import yaml
import numpy as np
#from  launch_ros.actions import ROSTimer



def generate_launch_description():
    # Get the name of config file of the current experiment
    specs = os.environ['PARAMS_FILE']
    # Opening the config file to take the experiment data such as spawn pose
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    controller_type=robot_specs['controller_type']
    planner_type=robot_specs['planner_type'] 
    instances_num= robot_specs['instances_num']
    if robot_specs['trajectory_type'] == 'user_defined':
        trajectories=robot_specs['user_defined_trajectories']
    elif robot_specs['trajectory_type'] == 'auto_generated':
        trajectories=robot_specs['auto_generated_trajectory']['types']

    # Node for generating pdf
    pdf_generator=Node(
            name='pdf_generator',
            executable='pdf_generator',
            package='ROSNavBench',
        )  
      
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
 
    trajectory_generator = Node(
            name='trajectory_generator',
            executable='trajectory_generator',
            package='ROSNavBench',
        )
  
    ld = LaunchDescription()
    ld.add_action(spawn_robot)
    ld.add_action(nav2)
    #ld.add_action(ROSTimer(period=5, actions=[trajectory_generator]))
    ld.add_action(trajectory_generator)
    ld.add_action(SetEnvironmentVariable(name='planner',value=planner_type[0]))
    ld.add_action(SetEnvironmentVariable(name='controller',value=controller_type[0]))
    ld.add_action(SetEnvironmentVariable(name='trajectory_num',value=str(0)))
    ld.add_action(SetEnvironmentVariable(name='round_num',value="1"))
    ld.add_action(SetEnvironmentVariable(name='iteration_id',value="0"))
    # Generating  different nodes to publish the type of running controller and planner 
    controller_node=[]
    for j in range(len(controller_type)*len(planner_type)*instances_num*len(trajectories)):
       controller_node.append(Node(
        name='marker_publisher',
        executable='marker_publisher',
        package='ROSNavBench',
    ))           
    # Generating different nodes for sending the goal and recording the data
    nodes=[]
    for j in range(len(controller_type)*len(planner_type)*instances_num*len(trajectories)):
       nodes.append(Node(
        name='follow_path_0',
        executable='follow_path',
        package='ROSNavBench',
    )) 
    # Generating different nodes for reseting the pose of the robot to intial pose after each controller scenario   
    state_nodes=[]
    for k in range(len(controller_type)*len(planner_type)*instances_num*len(trajectories)): 
        state_nodes.append(Node(
        name='reset_robot',
        executable='reset_robot',
        package='ROSNavBench',
    ))  

    for k in range(len(planner_type)):
        for i in range(len(controller_type)):
            for p in range(len(trajectories)):
                experiment_number = k * (len(controller_type) * len(trajectories)) + i * len(trajectories) + p + 1
                for q in range(instances_num):
                    if i==0 and k==0 and q==0 and p==0:
                        #add if condition to stop the procces if one of the user defined traj is not valid 
                        ld.add_action(RegisterEventHandler(OnProcessExit(target_action= trajectory_generator, on_exit=state_nodes[0])))
                        ld.add_action(RegisterEventHandler(OnProcessExit(target_action= state_nodes[0], on_exit=nodes[0])))
                        ld.add_action(RegisterEventHandler(OnProcessExit(target_action= state_nodes[0],on_exit=controller_node[0])))
                    else:
                        increament = q + p * instances_num + i * (len(trajectories) * instances_num) + k * (len(controller_type) * len(trajectories) * instances_num)-1
                        #increament=k*len(controller_type)*instances_num+i*instances_num+q-1
                        ld.add_action(RegisterEventHandler(OnProcessExit(target_action= nodes[increament], on_exit=[SetEnvironmentVariable(name='planner',value=planner_type[k]),SetEnvironmentVariable(name='controller',value=controller_type[i]),SetEnvironmentVariable(name='iteration_id',value=str(q)),SetEnvironmentVariable(name='trajectory_num',value=str(p)),SetEnvironmentVariable(name='round_num',value=str(experiment_number)),state_nodes[increament+1]]))) #edit 
                        ld.add_action(RegisterEventHandler(OnProcessExit(target_action=state_nodes[increament+1], on_exit=[nodes[increament+1],controller_node[increament+1]])))  
    
                    
    # Once all events are done, the node of generating a pdf will start
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=nodes[len(controller_type)*len(planner_type)*len(trajectories)*instances_num-1], on_exit=[pdf_generator] )))

    return ld
    
