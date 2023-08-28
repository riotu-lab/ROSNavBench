#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import numpy as np
import yaml
import os
import math
import psutil
import time 
import csv
from rclpy.impl import rcutils_logger


# Get the name of config file of the current experiment
params_file = os.environ['PARAMS_FILE']
rclpy.init()
global navigator
navigator = BasicNavigator()


def main(args=None): 

    '''
    This function is responsile to:
    1. set intial pose
    2. form a goal or a trajectory
    3. Send the goal with the specifed controller
    4. Record data
    5. Save data to csv file 
    '''

  
    # Opening the config file to take the experiment data such as spawn pose, and the goal pose or trjectory
    specs= os.path.join(
        get_package_share_directory('ROSNavBench'),
        'config',
        params_file+'.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    x = robot_specs['spawn_pose_x']     
    y = robot_specs['spawn_pose_y']        
    yaw = robot_specs['spawn_pose_yaw'] 
    trajectory_type= robot_specs['trajectory_type'] 
    pdf_name=robot_specs['experiment_name']
    controller_type=robot_specs['controller_type']
    behaviour_tree_directory=robot_specs['behaviour_tree_directory']
    # all operations will start once Nav2 is ready   
    navigator.waitUntilNav2Active()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = x
    #initial_pose.pose.orientation.z = np.sin(np.deg2rad(90)/2)
    initial_pose.pose.position.y = y
    initial_pose.pose.orientation.x =  0.0
    initial_pose.pose.orientation.y =  0.0 
    # Convert the eular angles to quaternion 
    initial_pose.pose.orientation.z = np.sin(yaw/2)  
    initial_pose.pose.orientation.w = np.cos(yaw/2)    
    if trajectory_type=='circle':
       r= robot_specs['radius']
       initial_pose.pose.position.x = x+r
       initial_pose.pose.orientation.w = np.cos(np.deg2rad(90)/2) 
       initial_pose.pose.orientation.z = np.sin(np.deg2rad(90)/2)
    
    navigator.setInitialPose(initial_pose)

    # A few gap of time is nesseray to make sure that the intial pose is set, and then clear the map
    time.sleep(5)
    navigator.clearAllCostmaps()
    navigator.getGlobalCostmap()
    navigator.getLocalCostmap()
    time.sleep(3)

    # Based on the type of the trajectory, the goal pose or poses will be formated
    goal_poses = []

    '''
    Square and circle trajectories are generated through provides a list of waypoints

    The concept is to find the length of each segment

    The segment must be samll. but not samll that the robot consider it as achieved without even moving
    
    The trajectory is not close, as if the last waypoint is same or close to the start point, 
    the robot will cnsider the goal achived without even moving
    '''

    if trajectory_type=='square':

       side_length=robot_specs['side_length']
       waypoints_array=[]
       array=[+x,+y,-x,-y]
       
       # Round to nearset number divisable by 0.5 and then find the number of segments
       # Each segment will be 0.5 m length, this distance is small enough to make the robot move 
       segments_num=(0.5*round(side_length/0.5))*2
       
       # The following 4 loops are to generate the waypoints for each side of the robot
       for i in range(int(segments_num)-1):
           waypoints_array.append([x+((i+1)*0.5),y])
       for k in range(int(segments_num)-1):
           waypoints_array.append([x+((i+1)*0.5),y+((k+1)*0.5)])  
       for t in range(int(segments_num)-1):
           waypoints_array.append([(x+(i+1)*0.5)-((t+1)*0.5),y+((k+1)*0.5)])           
       for q in range(int(segments_num)-2):
           waypoints_array.append([x,(y+(k+1)*0.5)-((q+1)*0.5)])  

       # Add the generated waypoints to the goal_poses list in term of goal form         
       for i in range(len(waypoints_array)):
           point=waypoints_array[i]
           goal_pose= PoseStamped()
           goal_pose.header.frame_id = 'map'
           goal_pose.header.stamp = navigator.get_clock().now().to_msg()
           goal_pose.pose.position.x = point[0]
           goal_pose.pose.position.y = point[1]
           goal_poses.append(goal_pose)
                                                 
    elif trajectory_type=='circle':
         # The robot is spawned in the center of the cirle
         # Thus, the robot will first move a raduis length to the circle circumference
         # This movment is not included in the report 
         r= robot_specs['radius']
        #  goal_pose = PoseStamped()
        #  goal_pose.header.frame_id = 'map'
        #  goal_pose.header.stamp = navigator.get_clock().now().to_msg()       
        #  goal_pose.pose.position.x = r+x
        #  goal_pose.pose.position.y = y
        #  goal_pose.pose.orientation.w = np.cos(np.deg2rad(90)/2) 
        #  goal_pose.pose.orientation.z = np.sin(np.deg2rad(90)/2)
        #  navigator.goToPose(goal_pose,behavior_tree=os.path.join(get_package_share_directory('turtlebot3_navigation2'),
        # 'param',
        # os.environ["controller"]+'.xml')) 
         

        # # The circel trajectory is sent once the robot arrives at the circumference  
        #  while not navigator.isTaskComplete():
        #     pass 
          
        # The segments number is the cumenference diveded by 0.1
        # The increament angle is then found based on the segments number. It is used to find the waypoints coordinates 
         circumference=2*math.pi*r
         segment_num=circumference/0.1
         increment_angle=360/segment_num  
         # Loop over the segments to generate the waypoints list
         # Note that 'segment_num-2' means the last two segments will be ignored. the reason is 
         # to have a distance between spawn pose and the last waypoint           
         for i in range(int(segment_num-2)):
         
             angle= (i+1)*increment_angle
             goal_pose= PoseStamped()
             goal_pose.header.frame_id = 'map'
             goal_pose.header.stamp = navigator.get_clock().now().to_msg()
             goal_pose.pose.position.x =r*np.cos(np.deg2rad(angle))+x
             goal_pose.pose.position.y = r*np.sin(np.deg2rad(angle))+y
             goal_poses.append(goal_pose)
         
            
    elif trajectory_type=='several_waypoints': 
         # The waypoints are put into the goal form and then added to goal_poses array
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
         # The goal is put into the goal form
         x_goal=robot_specs['goal_pose_x']     
         y_goal=robot_specs['goal_pose_y']
         yaw_goal=robot_specs['goal_pose_yaw'] 
         goal_pose = PoseStamped()
         goal_pose.header.frame_id = 'map'
         goal_pose.header.stamp = navigator.get_clock().now().to_msg()
         goal_pose.pose.position.x = x_goal
         goal_pose.pose.position.y = y_goal
         goal_pose.pose.orientation.z = np.sin(yaw_goal/2) 
         goal_pose.pose.orientation.w = np.cos(yaw_goal/2) 
                
 
    # Sending the goal pose or poses
    

 
    logger = rcutils_logger.RcutilsLogger(name="my_logger")
    logger.info("The controller is "+os.environ["controller"])
    # The behaviour tree is sent with the goal in order to specify the local planner
    if trajectory_type=='one_goal':
       navigator.goToPose(goal_pose,behavior_tree=os.path.join(behaviour_tree_directory,
        os.environ["controller"]+'.xml'))
       
       
    elif  trajectory_type=='several_waypoints' or trajectory_type=='circle' or trajectory_type=='square':
       navigator.goThroughPoses(goal_poses,behavior_tree=os.path.join(behaviour_tree_directory,
        os.environ["controller"]+'_poses.xml')) 
       
    
    #These varibales are used for formating the csv file that conatins the raw data 
    empty_matrix=["", "", "", "", "", "", ""]
    data=[["CPU usage %", "Memory usage\n%", "x-pose", "y-pose", 'number of\nrecoveries', 'distance\nremaining','navigation\ntime'],empty_matrix]

    # A time gap is used to make sudurationre the the goal is sent 
    time.sleep(0.1)
    
    # The while loop is used to record the data while the task is running
    i = 0
    while not navigator.isTaskComplete():

        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

        row=[]
        row.append(psutil.cpu_percent())
        row.append(psutil.virtual_memory().percent)
        row.append(round(feedback.current_pose.pose.position.x,2))
        row.append(round(feedback.current_pose.pose.position.y,2))
        row.append(feedback.number_of_recoveries)
        row.append(round(feedback.distance_remaining,2))
        row.append(round(Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9, 2))
        data.append(row)
        time.sleep(0.2)

    # Getting the result of task     
    result = navigator.getResult()    
    
    if result == TaskResult.SUCCEEDED:
        result1='succeeded'
    elif result == TaskResult.CANCELED:
        result1='canceled'
    elif result == TaskResult.FAILED:
        result1='failed'
    else:
        result1='goal has an invalid return status!' 
    data.append(result1)

    # Creating a .csv file that contains all the raw data about the task
    f=open(os.path.join(get_package_share_directory('ROSNavBench'),
        'raw_data',
        pdf_name+'_'+os.environ["controller"]+os.environ["round_num"]+'.csv'),'w')
    writer=csv.writer(f,quoting=csv.QUOTE_NONNUMERIC, delimiter=' ')
    writer.writerows(data)
   
    navigator.destroyNode()
    
    rclpy.shutdown()
    exit(0)



