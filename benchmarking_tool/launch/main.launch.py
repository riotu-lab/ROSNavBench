### Script Creator: Fathima AlAhmed
### Maintainer: Saqeef Tehnan Manna


### This launch file is the one to be used which includes multiple other 
### launch files like spawning of the robot, running navigation, recording data and sending goal.

# July 8th, 2023

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete,OnProcessExit,OnProcessStart
from launch.conditions import IfCondition

def generate_launch_description():

    spawn_robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
         FindPackageShare("benchmarking_tool"), '/launch', '/spawn_turtlebot3.launch.py'])
        
            )
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("benchmarking_tool"), '/launch', '/nav2.launch.py'])
 
            )
    record_data = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("benchmarking_tool"), '/launch', '/record_data.launch.py'])

            )    

    follow_path = Node(
                package = 'benchmarking_tool',
                executable = 'follow_path',                
            ) 
    
    generate_pdf= Node(
                package = 'benchmarking_tool',
                executable = 'generate_pdf', 
            ) 
          
    return LaunchDescription([
    spawn_robot,
    nav2,
    follow_path,
    
    ])
