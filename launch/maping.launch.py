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
params_file = os.environ['PARAMS_FILE']

def generate_launch_description():

    models_path=''     
    #Launch directory 
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    
    # Launch args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    #World 
    world_path = os.path.join(FindPackageShare(package='ROSNavBench').find('ROSNavBench'), 'world', 'edited_office.world')
    
    #Setting the gazebo model path

    gazebo_models_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models')
    gazebo_models_path += ':'+os.path.join(
        get_package_share_directory('ROSNavBench'),
        'model')
    if models_path!='':
       gazebo_models_path+=':'+models_path
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =   gazebo_models_path
    

    #URDF  arg
    urdf_file_name = 'waffle'
    urdf_package=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'))
    urdf = os.path.join(
        urdf_package,
        'urdf',
        'turtlebot3_'+urdf_file_name+'.urdf')
        
    #Model  arg
    model_package=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'))
    model_path = os.path.join(
        model_package,
        'models',
        'turtlebot3_'+urdf_file_name,
        'model.sdf')

    #Start Gazebo
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen')
            
    # start robot state publisher 
    publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        name = "robot_state_publisher",
        parameters = [{'use_sim_time': use_sim_time,
#                       'frame_prefix':namespace+'/',
                     'robot_description': Command(['xacro ', urdf])}],
        output = "screen",

    )

    # Spawn robot 
    spawn_node=  Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        output = 'screen',
        arguments = ['-entity','turtlebot3',
                   '-file',model_path,
#                   '-robot_namespace',namespace,
                   '-x', '3.775616', '-y', '6.273945', '-z', '0.01','-Y','0.010304','-R','0.000013','-P','0.001698']   
    )

    #Creating the launch description
    ld = LaunchDescription()   

    ld.add_action(gazebo) 
    ld.add_action(publisher_node)
    ld.add_action(spawn_node) 
    #ld.deprecated()


    return ld


