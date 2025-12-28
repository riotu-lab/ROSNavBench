
### This launch file is the one to be used which includes multiple other 
### launch files like spawning of the robot, running navigation, sending goal,recording data, and generating report.

# July 8th, 2023

import os
from sys import executable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import FindExecutable
import yaml
import numpy as np
from ROSNavBench.path_utils import resolve_paths_in_config
#from  launch_ros.actions import ROSTimer


def generate_launch_description():
    # Declare launch argument for params_file with default
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ROSNavBench'),
            'config',
            'house_experiment_no_obstaclesyaml.yaml'
        ]),
        description='Path to the configuration YAML file (can be relative to package or absolute)'
    )
    
    # Use OpaqueFunction to handle path resolution
    return LaunchDescription([
        params_file_arg,
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context):
    # Get the params_file path (from argument or environment variable for backward compatibility)
    if 'PARAMS_FILE' in os.environ:
        specs = os.environ['PARAMS_FILE']
    else:
        # Get from launch argument
        params_file_config = LaunchConfiguration('params_file')
        specs = params_file_config.perform(context)
    
    # Resolve relative paths if needed
    if not os.path.isabs(specs):
        # Try to resolve relative to package
        try:
            package_dir = get_package_share_directory('ROSNavBench')
            specs = os.path.join(package_dir, specs)
            specs = os.path.normpath(specs)
            specs = os.path.abspath(specs)
        except Exception:
            # If package not found, try relative to current working directory
            specs = os.path.abspath(specs)
    
    # Opening the config file to take the experiment data such as spawn pose
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
    
    # Resolve all paths in the config relative to the config file location
    resolve_paths_in_config(robot_specs, specs)
    
    # Set PARAMS_FILE environment variable with resolved path for child processes
    actions = [SetEnvironmentVariable(name='PARAMS_FILE', value=specs)]
        
    # Extract ALL values immediately to avoid storing dictionary references that launch might try to serialize
    controller_type = list(robot_specs['controller_type'])  # Make a copy
    planner_type = list(robot_specs['planner_type'])  # Make a copy
    instances_num = max(1, robot_specs['instances_num'])  # Ensure at least 1 instance
    
    # Extract spawn pose and trajectory count BEFORE any potential serialization
    trajectory_type = robot_specs['trajectory_type']
    if trajectory_type == 'user_defined':
        # Extract spawn pose values immediately, don't store dictionary references
        user_trajectories = robot_specs['user_defined_trajectories']
        trajectories_count = len(user_trajectories)
        first_traj = user_trajectories[0]
        sp = first_traj["spawn_pose"]
        x = str(sp['x'])
        y = str(sp['y'])
        yaw = str(sp['yaw'])
        # Clear references to avoid serialization
        del user_trajectories, first_traj, sp
    elif trajectory_type == 'auto_generated':
        # Extract spawn pose values immediately, don't store dictionary references
        auto_traj = robot_specs['auto_generated_trajectory']
        trajectories_count = len(auto_traj['types'])
        sp = auto_traj["spawn_pose"]
        x = str(sp['x'])
        y = str(sp['y'])
        yaw = str(sp['yaw'])
        # Clear references to avoid serialization
        del auto_traj, sp
    else:
        trajectories_count = 1
        x = '0.0'
        y = '0.0'
        yaw = '0.0'
    
    # Extract other needed values
    urdf_file = robot_specs.get('urdf_file', '')
    world_path = robot_specs.get('world_path', '')
    
    # Clear the robot_specs dictionary reference to prevent serialization issues
    del robot_specs

    # Node for generating pdf
    pdf_generator = Node(
        name='pdf_generator',
        executable='pdf_generator',
        package='ROSNavBench',
    )  
      
    # Include launch file for spawning the robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ROSNavBench"), '/launch', '/spawn_robot.launch.py']),
        launch_arguments={
            'x_pose': x,
            'y_pose': y,
            'yaw_pose': yaw, 
            'urdf_file': urdf_file,
            'world': world_path
        }.items()
    )

    # Include launch file for launching navigation
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ROSNavBench"), '/launch', '/nav2.launch.py']),
        launch_arguments={
            'x_pose': x,
            'y_pose': y,
            'yaw_pose': yaw,
            'use_sim_time': 'true'
        }.items()
    )      
 
    trajectory_generator = Node(
        name='trajectory_generator',
        executable='trajectory_generator',
        package='ROSNavBench',
        parameters=[{'use_sim_time': True}]
    )
  
    actions.extend([
        spawn_robot,
        nav2,
        trajectory_generator,
        SetEnvironmentVariable(name='planner', value=planner_type[0]),
        SetEnvironmentVariable(name='controller', value=controller_type[0]),
        SetEnvironmentVariable(name='trajectory_num', value=str(0)),
        SetEnvironmentVariable(name='round_num', value="1"),
        SetEnvironmentVariable(name='iteration_id', value="0")
    ])
    
    # Generating different nodes to publish the type of running controller and planner 
    controller_node = []
    for j in range(len(controller_type) * len(planner_type) * instances_num * trajectories_count):
        controller_node.append(Node(
            name='marker_publisher',
            executable='marker_publisher',
            package='ROSNavBench',
            parameters=[{'use_sim_time': True}],
        ))           
    
    # Generating different nodes for sending the goal and recording the data
    nodes = []
    for j in range(len(controller_type) * len(planner_type) * instances_num * trajectories_count):
        nodes.append(Node(
            name='follow_path_0',
            executable='follow_path',
            package='ROSNavBench',
            parameters=[{'use_sim_time': True}],
        )) 
    
    # Generating different nodes for reseting the pose of the robot to initial pose after each controller scenario   
    state_nodes = []
    for k in range(len(controller_type) * len(planner_type) * instances_num * trajectories_count): 
        state_nodes.append(Node(
            name='reset_robot',
            executable='reset_robot',
            package='ROSNavBench',
            parameters=[{'use_sim_time': True}],
        ))  

    for k in range(len(planner_type)):
        for i in range(len(controller_type)):
            for p in range(trajectories_count):
                experiment_number = k * (len(controller_type) * trajectories_count) + i * trajectories_count + p + 1
                for q in range(instances_num):
                    if i == 0 and k == 0 and q == 0 and p == 0:
                        #add if condition to stop the process if one of the user defined traj is not valid 
                        actions.append(RegisterEventHandler(OnProcessExit(target_action=trajectory_generator, on_exit=state_nodes[0])))
                        actions.append(RegisterEventHandler(OnProcessExit(target_action=state_nodes[0], on_exit=nodes[0])))
                        actions.append(RegisterEventHandler(OnProcessExit(target_action=state_nodes[0], on_exit=controller_node[0])))
                    else:
                        increament = q + p * instances_num + i * (trajectories_count * instances_num) + k * (len(controller_type) * trajectories_count * instances_num) - 1
                        #increament=k*len(controller_type)*instances_num+i*instances_num+q-1
                        actions.append(RegisterEventHandler(OnProcessExit(target_action=nodes[increament], on_exit=[SetEnvironmentVariable(name='planner', value=planner_type[k]), SetEnvironmentVariable(name='controller', value=controller_type[i]), SetEnvironmentVariable(name='iteration_id', value=str(q)), SetEnvironmentVariable(name='trajectory_num', value=str(p)), SetEnvironmentVariable(name='round_num', value=str(experiment_number)), state_nodes[increament+1]]))) #edit 
                        actions.append(RegisterEventHandler(OnProcessExit(target_action=state_nodes[increament+1], on_exit=[nodes[increament+1], controller_node[increament+1]])))  
    
    # Once all events are done, the node of generating a pdf will start
    # Only register if there are nodes to wait for
    if len(nodes) > 0:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=nodes[len(controller_type) * len(planner_type) * trajectories_count * instances_num - 1], on_exit=[pdf_generator])))
    else:
        # If no nodes, just add pdf_generator directly
        actions.append(pdf_generator)

    return actions
