from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

       
    ld = LaunchDescription() 
    ld.add_action(
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                '/set_entity_state ',
                'gazebo_msgs/SetEntityState ',
                '"state: {name: turtlebot3_waffle, pose: {position:{x: -6.28, y: -4.0, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0.7103532724176078 , w: 0.7038453156522361 }}, reference_frame: world}"'
            ]],
            shell=True
        )
    )

    return ld    
    

#ros2 topipub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: -6.28, y: -4.0, z: 0.0}, orientation: {z: 0.71, w: 0.70}}, } }'


