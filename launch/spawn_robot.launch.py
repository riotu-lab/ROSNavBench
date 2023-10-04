#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro
import yaml

# Get the name of config file of the current experiment
specs = os.environ['PARAMS_FILE']

def generate_launch_description():


    # Opening the config file to take the experiment data such as intial pose of spawned robot 
    # specs= os.path.join(
    #     get_package_share_directory('ROSNavBench'),
    #     'config',
    #     params_file+'.yaml'
    #    )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    x = robot_specs['spawn_pose_x']     
    y = robot_specs['spawn_pose_y']
    yaw = robot_specs['spawn_pose_yaw']
    world_path = robot_specs['world_path']  
    models_path=  robot_specs['models_path'] 
    urdf=robot_specs['urdf_file']
    model_path=robot_specs['model_file']
    trajectory_type= robot_specs['trajectory_type']
    
    if trajectory_type=='circle':
        r= robot_specs['radius']
        x=x+r
        yaw=1.5707963 # 90 degrees
    #Launch directory of gazebo_ros pkg
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    
    # Launch args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        
    gazebo_models_path=""
    if models_path!='':
       gazebo_models_path=models_path
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =   gazebo_models_path

    #URDF  arg
    # urdf_file_name = robot_specs['turtlebot3_model']
    # urdf_package=os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'))
    # urdf = os.path.join(
    #     urdf_package,
    #     'urdf',
    #     'turtlebot3_'+urdf_file_name+'.urdf')
      
    # #Model  arg
    # model_package=os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'))
    # model_path = os.path.join(
    #     model_package,
    #     'models',
    #     'turtlebot3_'+urdf_file_name,
    #     'model.sdf')

    #Start Gazebo
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen')
            
    # Start robot state publisher 
    publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        name = "robot_state_publisher",
        parameters = [{'use_sim_time': use_sim_time,
                       'robot_description': Command(['xacro ', urdf])}],
        output = "screen",

    )

    # Spawn robot either from a topic or a file
    if model_path=='None':
        spawn_node=  Node(
            package = 'gazebo_ros',
            executable = 'spawn_entity.py',
            output = 'screen',
            arguments = ['-entity','robot',
                   '-topic','robot_description',
                   '-x', str(x), '-y', str(y), '-z', '0.01','-Y',str(yaw)]   
        )
    else: 
        spawn_node=  Node(
            package = 'gazebo_ros',
            executable = 'spawn_entity.py',
            output = 'screen',
            arguments = ['-entity','robot',
                   '-file',model_path,
                   '-x', str(x), '-y', str(y), '-z', '0.01','-Y',str(yaw)]   
        )        


    #Creating the launch description
    ld = LaunchDescription()   

    ld.add_action(gazebo) 
    ld.add_action(publisher_node)
    ld.add_action(spawn_node) 
  
    return ld

