import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete,OnProcessExit,OnProcessStart


def generate_launch_description():

    spawn_robot=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("benchmarking_tool"), '/launch', '/spawn_turtlebot3.launch.py'])
        
            )
    nav2=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("benchmarking_tool"), '/launch', '/nav2.launch.py'])
 
            )
    record_data=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("benchmarking_tool"), '/launch', '/record_data.launch.py'])

            )    

    send_goal=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("benchmarking_tool"), '/launch', '/send_goal.launch.py'])

            ) 


    return LaunchDescription([
    spawn_robot,
    RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=spawn_robot,
            on_completion=[nav2],
        )
    ),
    RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=nav2,
            on_completion=[record_data],
        )
    ),
    RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=record_data,
            on_start=[send_goal],
        )
    ), 
          
    ])
