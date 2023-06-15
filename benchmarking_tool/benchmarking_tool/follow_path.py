#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import math

navigator= None 
def main():
    rclpy.init()
    global navigator
    navigator = BasicNavigator()

    #Intial pose of spawn robot of spawned turtlebot3 
    specs= os.path.join(
        get_package_share_directory('benchmarking_tool'),
        'config',
        'params.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    x = robot_specs['spawn_pose_x']     
    y = robot_specs['spawn_pose_y']        
    yaw = robot_specs['spawn_pose_yaw'] 
    trajectory_type= robot_specs['trajectory_type'] 
    x_goal=robot_specs['goal_pose_x']     
    y_goal=robot_specs['goal_pose_y']
    yaw_goal=robot_specs['goal_pose_yaw']  
    
    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = x
    initial_pose.pose.position.y = y
    initial_pose.pose.orientation.x =  0.0
    initial_pose.pose.orientation.y =   0.0
    initial_pose.pose.orientation.z = np.sin(yaw/2) 
    initial_pose.pose.orientation.w = np.cos(yaw/2) 
    navigator.setInitialPose(initial_pose)
    
    navigator.waitUntilNav2Active()
    result = navigator.getResult()

    # set our demo's goal poses
    goal_poses = []
    if trajectory_type=='square':

       side_length=robot_specs['side_length']
       waypoints_array=[]
       array=[+x,+y,-x,-y]
       segments_num=round(side_length)*2
       for i in range(segments_num-1):
           print("anything")
           waypoints_array.append([x+((i+1)*0.5),y])
       for k in range(segments_num-1):
           waypoints_array.append([x+((i+1)*0.5),y+((k+1)*0.5)])  
       for t in range(segments_num-1):
           waypoints_array.append([(x+(i+1)*0.5)-((t+1)*0.5),y+((k+1)*0.5)])           
       for q in range(segments_num-2):
           waypoints_array.append([x,(y+(k+1)*0.5)-((q+1)*0.5)])           
       for i in range(len(waypoints_array)):
           point=waypoints_array[i]
           goal_pose= PoseStamped()
           goal_pose.header.frame_id = 'map'
           goal_pose.header.stamp = navigator.get_clock().now().to_msg()
           goal_pose.pose.position.x = point[0]
           goal_pose.pose.position.y = point[1]
           goal_poses.append(goal_pose)
       
       print(waypoints_array)   
       #print(goal_poses)                                          
    elif trajectory_type=='circle':
         r= robot_specs['radius']
         goal_pose = PoseStamped()
         goal_pose.header.frame_id = 'map'
         goal_pose.header.stamp = navigator.get_clock().now().to_msg()       
         goal_pose.pose.position.x = r+x
         goal_pose.pose.position.y = y
         goal_pose.pose.orientation.w = np.cos(np.deg2rad(90)/2) 
         goal_pose.pose.orientation.z = np.sin(np.deg2rad(90)/2)
         navigator.goToPose(goal_pose) 
         
         circumference=2*math.pi*r
         segment_num=circumference/0.1
         increment_angle=360/segment_num  
         while not navigator.isTaskComplete():
            pass 
             
         for i in range(int(segment_num-2)):
         
             angle= (i+1)*increment_angle
             goal_pose= PoseStamped()
             goal_pose.header.frame_id = 'map'
             goal_pose.header.stamp = navigator.get_clock().now().to_msg()
             goal_pose.pose.position.x =r*np.cos(np.deg2rad(angle))+x
             goal_pose.pose.position.y = r*np.sin(np.deg2rad(angle))+y
             goal_poses.append(goal_pose)
         
            
    elif trajectory_type=='several_waypoints': 
         waypoints= robot_specs['waypoints']
         for i in range(len(waypoints)):
            goal=waypoints[i]

            goal_pose= PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x =goal[0]
            goal_pose.pose.position.y = goal[1]
            goal_pose.pose.orientation.w = np.cos(goal[2]/2) 
            goal_pose.pose.orientation.z = np.sin(goal[2]/2) 
            goal_poses.append(goal_pose)
            print(goal_poses)
            
    elif trajectory_type=='one_goal': 
         goal_pose = PoseStamped()
         goal_pose.header.frame_id = 'map'
         goal_pose.header.stamp = navigator.get_clock().now().to_msg()
         goal_pose.pose.position.x = x_goal
         goal_pose.pose.position.y = y_goal
         goal_pose.pose.orientation.z = np.sin(yaw_goal/2) 
         goal_pose.pose.orientation.w = np.cos(yaw_goal/2) 
                

    if trajectory_type=='one_goal':
       navigator.goToPose(goal_pose)
    elif  trajectory_type=='several_waypoints' or trajectory_type=='circle' or trajectory_type=='square':
          navigator.goThroughPoses(goal_poses) 

    # sanity check a valid path exists
    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

    navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # RECORDING DIFFERENT DATA FROM FEEDBACK METHOD 
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            #if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #    navigator.cancelTask()



    # Do something depending on the return code
    #result = navigator.getResult()
    #if result == TaskResult.SUCCEEDED:
    #    print('Goal succeeded!')
    #elif result == TaskResult.CANCELED:
    #    print('Goal was canceled!')
    #elif result == TaskResult.FAILED:
    #    print('Goal failed!')
    #else:
    #    print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
