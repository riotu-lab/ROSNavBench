#! /usr/bin/env python3
from operator import mod
from turtle import circle
from typing import List
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.context import Context
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
from rclpy.parameter import Parameter
from std_msgs.msg import String
from  rcl_interfaces.msg import Log
from rclpy.time import Time
import xml.etree.ElementTree as ET
from sensor_msgs.msg import LaserScan
import pandas as pd
import subprocess
from ROSNavBench.extract_data import extract_data
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
from datetime import datetime
from nav_msgs.msg import Path
# Get the name of config file of the current experiment
specs = os.environ['PARAMS_FILE']


from lifecycle_msgs.srv import GetState  # Replace with the actual service type you need

class SimpleServiceWaiterNode(Node):
    def __init__(self):
        super().__init__('simple_service_waiter_node')
        self.client = self.create_client(GetState, 'amcl/get_state')  # Replace SetBool with your service type
        self.get_logger().info('Waiting for the amcl/set_state service to become available...')
        
        # Wait for the service to become available.
        self.client.wait_for_service()
        self.get_logger().info('Service is now available. Exiting node.')

class DataCollectionNode(Node):
    def __init__(self, navigator):
        super().__init__('data_collection_node')
        self.navigator = navigator
        # Instantiate subscribers
        # self.subscriber = LogSubscriber()
        # self.laser_reading = LaserSubscriber()
        # Data structures for storing collected data
        self.x_pose = []
        self.y_pose = []
        self.recoveries = []
        self.error_msgs=[]
        self.CPU=[]
        self.memory=[]
        self.time_stamp=[]
        self.distance_to_obstacles=[]
        self.log_msgs = None
        self.log_msg = None
        self.log_time = None
        self.log_level = None
        self.log_msg_name = None
        self.min_val = None
        #self.error_msg=[]
        self.log_msgs = None
        self.log_msg = None
        self.times_laser=[]
        self.publisher_node=[]
        self.msg_level=[]
        self.result=[]
        self.plan=None
        self.plan_list=[]
        self.log_subscription=self.create_subscription(
            Log,
            'rosout',
            self.LogSubscriber_callback,
            10
        )
        self.laser_subscription=self.create_subscription(
            LaserScan,
            'scan',
            self.LaserSubscribe_callback,
            10
        )
        self.global_plan_subscription=self.create_subscription(
            Path,
            'plan',
            self.PlanSubscribe_callback,
            10
        )
        # Initialize callback groups

        self.navigator_feedback_group = ReentrantCallbackGroup()
        # self.laser_reading_group = ReentrantCallbackGroup()
        # self.log_msg_group = ReentrantCallbackGroup()
        # self.computer_performance_group = ReentrantCallbackGroup()
      
        # Initialize timers for periodic callbacks
        self.create_timer(2, self.collect_navigator_feedback, callback_group=self.navigator_feedback_group)
       
    def PlanSubscribe_callback(self,msg):
        self.plan=msg

    def LaserSubscribe_callback(self,msg):
        #self.get_logger().info(f"Received LaserScan message: min_val = {datetime.now()}")
        self.min_val = min(msg.ranges)
        self.times_laser.append(msg.header.stamp.sec)

    def LogSubscriber_callback(self,msg):
        self.log_msgs = msg.msg
        self.log_time = msg.stamp
        self.log_msg_name = msg.name
        if msg.level==10:
            self.log_level = "DEBUG"
        elif msg.level==20:
            self.log_level = "INFO"
        elif msg.level==30:
            self.log_level = "WARN"
        elif msg.level==40:
            self.log_level = "ERROR"
        elif msg.level==50:
            self.log_level = "FATAL"

    def collect_navigator_feedback(self):
        #self.get_logger().info("Feedback time"+str(datetime.now()))
        self.CPU.append(psutil.cpu_percent())
        self.memory.append(psutil.virtual_memory().percent)
        self.distance_to_obstacles.append(self.min_val)
        self.error_msgs.append([self.log_msg_name,self.log_level,self.log_msgs])
        self.plan_list.append(self.plan)
        feedback = self.navigator.getFeedback()
        if feedback is not None:
            self.x_pose.append(round(feedback.current_pose.pose.position.x, 2))
            self.y_pose.append(round(feedback.current_pose.pose.position.y, 2))
            self.recoveries.append(feedback.number_of_recoveries)
            self.time_stamp.append(round(Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9, 2))
        else:
            self.x_pose.append(None)
            self.y_pose.append(None)
            self.recoveries.append(None)
        self.result.append("In progress")

    def collect_computer_performance(self):
        # Your computer performance collection logic here
        self.CPU.append(psutil.cpu_percent())
        self.memory.append(psutil.virtual_memory().percent)
        #self.get_logger().info("CPU"+str(datetime.now()))
    def get_collected_data(self):
        # Method to retrieve collected data
        return {
            "x_pose": self.x_pose,
            "y_pose": self.y_pose,
            "recoveries": self.recoveries,
            "error_msgs":self.error_msgs,
            "CPU":self.CPU,
            "memory":self.memory,
            "time_stamp":self.time_stamp,
            "distance_to_obstacles":self.distance_to_obstacles, 
            "result":self.result,
            "plan":self.plan_list
        }
    


def log_trail_info(path, experiment_id, iteration_id, trajectory_type, planner, controller, cpu_usage, memory_usage, navigation_time, x_pose, y_pose, x_path_plan, y_path_plan, number_of_recoveries, distance_to_obstacles, final_result, error_messages):
    # Calculate the maximum length among the different groups
    max_length = max(len(cpu_usage), len(x_path_plan), len(error_messages))

    # Create a list of dictionaries, each for one point in the longest group
    rows = []
    for i in range(max_length):
        # Safely access each element, considering the length of each list
        row_data = {
            'Experiment_ID': experiment_id,
            'Iteration_ID':iteration_id,
            'Trajectory_Type': trajectory_type,
            'Planner': planner,
            'Controller': controller,
            'Navigation_time': navigation_time[i] if i < len(navigation_time) else None,
            'CPU_Usage': cpu_usage[i] if i < len(cpu_usage) else None,
            'Memory_Usage': memory_usage[i] if i < len(memory_usage) else None,
            'x_pose': x_pose[i] if i < len(x_pose) else None,
            'y_pose': y_pose[i] if i < len(y_pose) else None,
            'x_path_plan': x_path_plan[i] if i < len(x_path_plan) else None,
            'y_path_plan': y_path_plan[i] if i < len(y_path_plan) else None,
            'number_of_recoveries': number_of_recoveries[i] if i < len(number_of_recoveries) else None,
            'distance_to_obstacles': distance_to_obstacles[i] if i < len(distance_to_obstacles) else None,
            'result': final_result[i] if i < len(final_result) else None, 
            'publisher_node':error_messages[i][0] if i < len(error_messages) and error_messages[i][0] else None,
            'msg_level':error_messages[i][1] if i < len(error_messages) and error_messages[i][1] else None,
            'Error_msgs': error_messages[i][2] if i < len(error_messages) and error_messages[i][2] else None
        }
        rows.append(row_data)

    # Convert list of dictionaries to DataFrame
    df = pd.DataFrame(rows)

    # Append to CSV, create if doesn't exist
    df.to_csv(path, mode='a', header=not pd.io.common.file_exists(path), index=False)

def find_closest_point(current_pose, path):
    """
    Finds the closest point on the path to the current pose.

    :param current_pose: Tuple (x, y) representing the current position.
    :param path: Tuple of two lists ([x], [y]) representing the path points.
    :return: Tuple (x, y) of the closest point on the path.
    """
    x_current, y_current = current_pose
    x_path, y_path = path

    # Calculate the Euclidean distances from the current pose to each point on the path
    distances = [(x - x_current)**2 + (y - y_current)**2 for x, y in zip(x_path, y_path)]

    # Find the index of the smallest distance
    min_index = distances.index(min(distances))

    # Return the closest point
    return x_path[min_index], y_path[min_index]




    
def main(args=None): 
    
    '''
    This function is responsile to:
    1. set intial pose
    2. form a goal or a trajectory
    3. Send the goal with the specifed controller
    4. Record data
    5. Save data to csv file 
    '''
    # The behaviour tree is sent with the goal in order to specify the local planner
    def modify_xml_path_planner(xml_file, new_planner_value,new_controller_value):
        # Load the XML file
        tree = ET.parse(xml_file)
        root = tree.getroot()

        # Find the element with the 'planner_id' attribute within the ComputePathToPose element
        if trajectory_type=="single_goal" or trajectory_type=='short_trajectory' or trajectory_type=='long_trajectory':
            element_with_planner_id = root.find(".//ComputePathToPose[@planner_id]")
        else:
            element_with_planner_id = root.find(".//ComputePathThroughPoses[@planner_id]")
        element_with_controller_id=root.find(".//FollowPath[@controller_id]")
        
        # Modify the 'planner_id' attribute value
        element_with_planner_id.set('planner_id', new_planner_value)
        element_with_controller_id.set('controller_id',new_controller_value)
        # Save the modified XML back to the file
        tree.write(os.path.join(behaviour_tree_directory,
        'bt_'+new_planner_value+'_'+new_controller_value+'.xml'))
    
    specs = os.environ['PARAMS_FILE']
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
    trajectory_type= robot_specs['trajectory_type'] 
    pdf_name=robot_specs['experiment_name']
    controller_type=robot_specs['controller_type']
    behaviour_tree_directory=robot_specs['behaviour_tree_directory']
    planner=os.environ["planner"]
    controller=os.environ["controller"]
    trajectory_num=os.environ["trajectory_num"]
    round_num = os.environ["round_num"]
    iteration_id=os.environ["iteration_id"]
    # Example values for x, y, yaw
    csv_path=os.path.join(get_package_share_directory('ROSNavBench'),
        'raw_data',pdf_name+'_trajectories.csv')
    trajectory_type, _initial_pose,trajectory_points=extract_data(csv_path,int(trajectory_num))
    rclpy.init()
    WaitNode = SimpleServiceWaiterNode()
    
    WaitNode.destroy_node()
    navigator = BasicNavigator()
    
    
    # all operations will start once Nav2 is ready  
    navigator.waitUntilNav2Active()
    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.position.x =  _initial_pose[0]
    initial_pose.pose.position.y =  _initial_pose[1]
    initial_pose.pose.orientation.x =  0.0
    initial_pose.pose.orientation.y =  0.0 
    initial_pose.pose.orientation.z =  _initial_pose[2] 
    initial_pose.pose.orientation.w =  _initial_pose[3]
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()

    navigator.setInitialPose(initial_pose)
    
    #navigator= 'bt_navigator', localizer='amcl'
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
  
  
    # Sending the goal pose or poses
    logger = rcutils_logger.RcutilsLogger(name="controller_type")
    logger.info("The controller is "+os.environ["controller"])
    
    # The behaviour tree is sent with the goal in order to specify the local planner
    if trajectory_type=='single_goal' or trajectory_type=='short_trajectory' or trajectory_type=='long_trajectory':
       
        modify_xml_path_planner(os.path.join(behaviour_tree_directory,
        'pose.xml'),os.environ["planner"],os.environ["controller"])
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x =  trajectory_points[0][0]
        goal_pose.pose.position.y =  trajectory_points[0][1]
        goal_pose.pose.orientation.x =  0.0
        goal_pose.pose.orientation.y =  0.0 
        goal_pose.pose.orientation.z =  trajectory_points[0][2] 
        goal_pose.pose.orientation.w =  trajectory_points[0][3]
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        time.sleep(0.2)
        path = navigator.getPath(initial_pose, goal_pose,planner_id=planner, use_start=True) 
        navigator.goToPose(goal_pose,behavior_tree=os.path.join(behaviour_tree_directory,
        'bt_'+os.environ["planner"]+'_'+os.environ["controller"]+'.xml'))      
    elif  trajectory_type=='waypoints' or trajectory_type=='circle' or trajectory_type=='square':
        modify_xml_path_planner(os.path.join(behaviour_tree_directory,
        'poses.xml'),os.environ["planner"],os.environ["controller"])
        time.sleep(0.2)
        for i in range(len(trajectory_points)):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x =  trajectory_points[i][0]
            goal_pose.pose.position.y =  trajectory_points[i][1]
            goal_pose.pose.orientation.x =  0.0
            goal_pose.pose.orientation.y =  0.0 
            goal_pose.pose.orientation.z =  trajectory_points[i][2] 
            goal_pose.pose.orientation.w =  trajectory_points[i][3]
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose)
        path = navigator.getPathThroughPoses(initial_pose, goal_poses,planner_id=planner, use_start=True) 
        navigator.goThroughPoses(goal_poses,behavior_tree=os.path.join(behaviour_tree_directory,
        'bt_'+os.environ["planner"]+'_'+os.environ["controller"]+'.xml')) 
       
       
    
    #These varibales are used for formating the csv file that conatins the raw data 
    
    
    # A time gap is used to make sudurationre the the goal is sent 
    time.sleep(0.1)
    
  
    
    def check_task_completion(executor, navigator):
        while not navigator.isTaskComplete():
            time.sleep(0.1)  # Check periodically, adjust the sleep duration as needed
        executor.shutdown()  # Stop the executor spinning when the task is complete
    #########
    
    
    data_collection_node = DataCollectionNode(navigator)
    #log_subscriber = LogSubscriber()
    #laser_subscriber = LaserSubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(data_collection_node)
    #executor.add_node(log_subscriber)
    #executor.add_node(laser_subscriber)

    task_check_thread = threading.Thread(target=check_task_completion, args=(executor, navigator))
    task_check_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        collected_data = data_collection_node.get_collected_data()
        data_collection_node.destroy_node()
    # while not navigator.isTaskComplete():
    #     executor.spin_once()
    #     time.sleep(1)
    x_pose = collected_data["x_pose"]  # This is a list
    y_pose = collected_data["y_pose"]  # This is another list
    distance_to_obstacles = collected_data["distance_to_obstacles"] 
    error_msgs = collected_data["error_msgs"]
    CPU = collected_data["CPU"]
    memory = collected_data["memory"]
    time_stamp = collected_data["time_stamp"]
    recoveries = collected_data["recoveries"]
    results = collected_data["result"]
    plan = collected_data["plan"]
    task_check_thread.join()  # Ensure the task check thread has finished
  
    # Getting the result of task     
    result = navigator.getResult()   
    if result == TaskResult.SUCCEEDED:
        results[-1] = 'succeeded'
    elif result == TaskResult.CANCELED:
        results[-1] = 'canceled'
    elif result == TaskResult.FAILED:
        results[-1] = 'failed'
    else:
        results[-1] = 'goal has an invalid return status!' 
    
    
    def clear_error_msgs(error_messages):
        for i in range(len(error_messages)):
            
            if error_messages[i][0] in ["controller_server", "behavior_server", "smoother_server", "planner_server", "bt_navigator", "waypoint_follower", "velocity_smoother", "lifecycle_manager_navigation", "map_server", "amcl", "lifecycle_manager_localization"]:
                if error_messages[i][1] in ["ERROR", "FATAL"]:
                    pass
                else:
                    error_messages[i] = [
                        None,
                        None,
                        None
                    ]
            else:
                error_messages[i] = [
                    None,
                    None,
                    None
                ]
        return error_messages
                    
    # # Example usage
    # current_pose = (1, 2)
    # path = ([2, 3, 1], [2, 3, 4])

    # closest_point = find_closest_point(current_pose, path)
    # closest_point
    def path_coordinates_extraction(path_msg):
        x_values = [pose.pose.position.x for pose in path_msg.poses]
        y_values = [pose.pose.position.y for pose in path_msg.poses]
        return x_values, y_values
    
    global_path_x=[]
    global_path_y=[]
    logger.info("The path is "+str(plan.count(None)))
    for i in range(len(x_pose)):
        if i==0 and plan[0]==None:
            plan[0]=path
        x,y=find_closest_point((x_pose[i],y_pose[i]), (path_coordinates_extraction(plan[i])))
        global_path_x.append(x)
        global_path_y.append(y)


    # for pose_stamped in path.poses:
    #     global_path_x.append(pose_stamped.pose.position.x)
    #     global_path_y.append(pose_stamped.pose.position.y)
    
    log_trail_info(os.path.join(get_package_share_directory('ROSNavBench'),'raw_data',
        pdf_name+'.csv'),round_num,str(int(iteration_id)+1), trajectory_type+"_"+trajectory_num, planner, controller,CPU, memory, time_stamp, x_pose, y_pose, global_path_x, global_path_y,recoveries,distance_to_obstacles,results,clear_error_msgs(error_msgs))
   

    navigator.destroyNode()
    
    rclpy.shutdown()
    exit(0)



