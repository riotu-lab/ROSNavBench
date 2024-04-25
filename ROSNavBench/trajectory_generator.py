from cgi import print_arguments
import posix
from typing_extensions import final
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
import os
import rclpy
import random
import numpy as np
from geometry_msgs.msg import PoseStamped
import math                  
from transforms3d.euler import euler2quat, quat2euler 
from PIL import Image 
import pandas as pd
from ament_index_python.packages import get_package_share_directory
import csv
import time 

rclpy.init()    
navigator = BasicNavigator()
navigator.waitUntilNav2Active()
# Opening the config file to take the experiment data such as spawn pose, and the goal pose or trjectory
specs = os.environ['PARAMS_FILE']
with open(specs, 'r') as file:
    robot_specs = yaml.safe_load(file)
map_png_path=robot_specs['map_png_path']
map_path=robot_specs['map_path']
planners_id_list=robot_specs['planner_type']

def circle_points(x, y, radius):
    """
    Generate waypoints for a circle trajectory.

    :param x: X-coordinate of the circle's center.
    :param y: Y-coordinate of the circle's center.
    :param radius: Radius of the circle.
    :return: List of waypoints (x, y) along the circle.
    """
    waypoints_array = []
    circumference = 2 * math.pi * radius
    segment_num = circumference / 0.1
    increment_angle = 360 / segment_num

    # Generate waypoints for the circle
    # Note that 'segment_num-2' means the last 
    # two segments will be ignored. the reason is 
    # to have a distance between spawn pose and the last waypoint 
    for i in range(int(segment_num - 2)):
        angle = (i + 1) * increment_angle
        waypoint_x = radius * np.cos(np.deg2rad(angle)) + x
        waypoint_y = radius * np.sin(np.deg2rad(angle)) + y
        waypoints_array.append([waypoint_x, waypoint_y])

    return waypoints_array

def square_points(x, y, side_length):
    """
    Generate waypoints for a square trajectory.

    :param x: X-coordinate of the square's starting point.
    :param y: Y-coordinate of the square's starting point.
    :param side_length: Length of the side of the square.
    :return: List of waypoints (x, y) forming the square.
    """
    waypoints_array = []
    # Round to nearset number divisable by 0.5 and then find the number of segments
    # Each segment will be 0.5 m length, this distance is small enough to make the robot move
    segments_num = int(0.5 * round(side_length / 0.5)) * 2

    # Generate waypoints for each side of the square
    for i in range(segments_num ):
        waypoints_array.append([x + ((i + 1) * 0.5), y])

    for k in range(segments_num ):
        waypoints_array.append([x+((i+1)*0.5),y+((k+1)*0.5)]) 

    for t in range(segments_num ):
        waypoints_array.append([(x+(i+1)*0.5)-((t+1)*0.5),y+((k+1)*0.5)]) 

    for q in range(segments_num -1):
        waypoints_array.append([x,(y+(k+1)*0.5)-((q+1)*0.5)]) 

    return waypoints_array
# Process user-defined trajectory
def process_user_defined_trajectory(goal_data: dict,robot_specs):
    trajectory_type = goal_data.get('type', {})
    trajectory_object=[]
    if trajectory_type=='circle':
        radius=goal_data.get('circle', {}).get('radius', 'default_radius')  
        x_initial = goal_data.get('circle_center').get('x')+radius
        y_initial = goal_data.get('circle_center').get('y')
        yaw_initial = np.deg2rad(90) 
    else:    
        x_initial = goal_data.get('spawn_pose').get('x')
        y_initial = goal_data.get('spawn_pose').get('y')
        yaw_initial = goal_data.get('spawn_pose').get('yaw')  
    initial_pose_ = initial_pose(x_initial,y_initial,yaw_initial)
    
    if trajectory_type=='square':
       side_length =goal_data.get('square', {}).get('side_length', 'default_side_length') 
       waypoints_array=square_points(x_initial,y_initial,side_length)       
       for i in range(len(waypoints_array)):
           point=waypoints_array[i]
           goal_pose= PoseStamped()
           goal_pose.header.frame_id = 'map'
           goal_pose.pose.position.x = point[0]
           goal_pose.pose.position.y = point[1]
           trajectory_object.append(goal_pose)
                                               
    elif trajectory_type == 'circle':
        # The robot is spawned at y and r+x 
        r = goal_data.get('circle', {}).get('radius', 'default_radius')  
        x_center = goal_data.get('circle_center').get('x')
        y_center = goal_data.get('circle_center').get('y')
        waypoints_array = circle_points(x_center,y_center,r)
        for i in range(len(waypoints_array)):  
            goal_pose= PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = waypoints_array[i][0]
            goal_pose.pose.position.y = waypoints_array[i][1]
            trajectory_object.append(goal_pose)
            
        
    elif trajectory_type == 'waypoints': 
         # The waypoints are put into the goal form and then added to goal_poses array
         waypoints= goal_data.get('waypoints', {}) 
         for i in range(len(waypoints)):
            goal=waypoints[i]
            goal_pose= PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = goal.get('x',{})
            goal_pose.pose.position.y = goal.get('y',{})
            goal_pose.pose.orientation.w = np.cos(goal.get('theta',{})/2) 
            goal_pose.pose.orientation.z = np.sin(goal.get('theta',{})/2) 
            trajectory_object.append(goal_pose)
           
    elif trajectory_type=='single_goal': 
         # The goal is put into the goal form
         x_goal= goal_data.get('single_goal', {}).get('x', 'default_x')    
         y_goal= goal_data.get('single_goal', {}).get('y', 'default_y')
         yaw_goal=goal_data.get('single_goal', {}).get('theta', 'default_theta')
         goal_pose = PoseStamped()
         goal_pose.header.frame_id = 'map'
         goal_pose.pose.position.x =  x_goal
         goal_pose.pose.position.y =  y_goal
         goal_pose.pose.orientation.z = np.sin(yaw_goal/2) 
         goal_pose.pose.orientation.w = np.cos(yaw_goal/2) 
         trajectory_object=goal_pose
  
    # validate trajectories and execlude the un required one 
    validation,path = validate_path(initial_pose_,trajectory_object, trajectory_type, navigator,planners_id_list)
    if validation == True:
            
        initial_pose_points=[initial_pose_.pose.position.x,initial_pose_.pose.position.y,initial_pose_.pose.orientation.z ,initial_pose_.pose.orientation.w ]
        trajectory_points=[]
        if trajectory_type=='single_goal': 
            trajectory_points.append([trajectory_object.pose.position.x,trajectory_object.pose.position.y,trajectory_object.pose.orientation.z ,trajectory_object.pose.orientation.w])
        else:
            for x in range(len(trajectory_object)):
                trajectory_points.append([trajectory_object[x].pose.position.x,trajectory_object[x].pose.position.y,trajectory_object[x].pose.orientation.z ,trajectory_object[x].pose.orientation.w])
    else: 
        initial_pose_points=None
        trajectory_points=None 
        raise ValueError(f"The path is not valid for {trajectory_type}")
    
    return [initial_pose_points,trajectory_points, trajectory_type]
    
         
# Generate a random trajectory within the specified bounds
def generate_random_trajectory(traj,navigator,planner_id):
    
    waypoints = []
    max_cost = 230
    # Adapted from navigation2 planner benchmarking metrics
    # Source: https://github.com/ros-planning/navigation2/blob/main/tools/planner_benchmarking/metrics.py
    costmap_msg = navigator.getGlobalCostmap()
    costmap = np.asarray(costmap_msg.data)
    costmap.resize(costmap_msg.metadata.size_y, costmap_msg.metadata.size_x)
    res = costmap_msg.metadata.resolution
    # ____________________________________________________________________________________________________
    
    side_buffer = int(max(costmap.shape[0]*0.15,costmap.shape[1]*0.10)) 
    initial_pose = getRandomStart(costmap, max_cost, side_buffer, res)
    goal_pose = getRandomGoal(costmap, initial_pose, max_cost, side_buffer, res)

    trajectory_type = traj.get('type', {})
    waypoints_poses=[]
    while True:
        validation,path=validate_path(initial_pose, goal_pose, 'single_goal', navigator,planner_id)
        
        if validation == True:
            path_length=calculate_path_length(path)
            if trajectory_type=='long_trajectory':
                if path_length<traj.get('long_trajectory', {}).get('lower_bound', 'default_lower_bound'):
                   goal_pose = getRandomGoal(costmap, initial_pose, max_cost, side_buffer,  res)
                else:
                    return [initial_pose, goal_pose],trajectory_type
            elif trajectory_type=='short_trajectory':
                if path_length>traj.get('short_trajectory', {}).get('upper_bound', 'default_lower_bound'):
                   goal_pose = Goal(costmap, initial_pose, max_cost, side_buffer,  res) 
                else:
                    return [initial_pose, goal_pose],trajectory_type
            elif trajectory_type=='waypoints':
                waypoints_poses.append(initial_pose)
                if len(waypoints_poses) == traj.get('waypoints', 'default_waypoints'):
                    return waypoints_poses,trajectory_type
                initial_pose = goal_pose
                goal_pose = getRandomGoal(costmap, initial_pose, max_cost, side_buffer,  res)
        elif validation == False:
            initial_pose = Start(costmap, max_cost, side_buffer,res)   
            goal_pose = getRandomGoal(costmap, initial_pose, max_cost, side_buffer,  res)

         
                
            
def set_intial_state(initial_pose,navigator):
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)

# Function edited copy from https://github.com/ros-planning/navigation2/blob/main/tools/planner_benchmarking/metrics.py
def getRandomStart(costmap, max_cost, side_buffer,  res):
    start = PoseStamped()
    start.header.frame_id = 'map'
   
    while True:
      
        row = random.randint(side_buffer, costmap.shape[0] - side_buffer)
        col = random.randint(side_buffer, costmap.shape[1] - side_buffer)

        if costmap[row, col] < max_cost:
            start.pose.position.x = col * res
            start.pose.position.y = row * res

            yaw = random.uniform(0, 1) * 2 * math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            start.pose.orientation.w = quad[0]
            start.pose.orientation.x = quad[1]
            start.pose.orientation.y = quad[2]
            start.pose.orientation.z = quad[3]
            break
    return start

# Function edited copy from https://github.com/ros-planning/navigation2/blob/main/tools/planner_benchmarking/metrics.py
def getRandomGoal(costmap, start, max_cost, side_buffer,  res):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
  
    while True:
    
        row = random.randint(side_buffer, costmap.shape[0] - side_buffer)
        col = random.randint(side_buffer, costmap.shape[1] - side_buffer)

        start_x = start.pose.position.x
        start_y = start.pose.position.y
        goal_x = col * res
        goal_y = row * res
        x_diff = goal_x - start_x
        y_diff = goal_y - start_y
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2)

        if costmap[row, col] < max_cost and dist > 3.0:
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            yaw = random.uniform(0, 1) * 2 * math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            goal.pose.orientation.w = quad[0]
            goal.pose.orientation.x = quad[1]
            goal.pose.orientation.y = quad[2]
            goal.pose.orientation.z = quad[3]
            break
    return goal

def initial_pose(x,y,yaw):
    _initial_pose = PoseStamped()
    _initial_pose.header.frame_id = 'map'
    _initial_pose.pose.position.x = x
    _initial_pose.pose.position.y = y
    _initial_pose.pose.orientation.x =  0.0
    _initial_pose.pose.orientation.y =  0.0 
    _initial_pose.pose.orientation.z = np.sin(yaw/2)  
    _initial_pose.pose.orientation.w = np.cos(yaw/2)   
    return _initial_pose

# Validate path using navigator.getPath()
def validate_path(initial_pose, waypoints, type, navigator,planner_id_list):
    # Implement validation logic based on your navigator's capabilities
    if type == 'square' or type == 'circle' or type == 'waypoints':
        # Maybe iterate over several planner to validate all
        for planner_id in planner_id_list:
            path = navigator.getPathThroughPoses(initial_pose, waypoints,planner_id=planner_id, use_start=True) 
            if path is None: 
                return False,None
        return True,path
    elif type == 'single_goal':
        for planner_id in planner_id_list:
            path = navigator._getPathImpl(initial_pose, waypoints,planner_id=planner_id,use_start=True) 
            if path is None: 
                return False, None
            
        return True,path
def euclidean_distance(pose1, pose2):
    x1, y1 = pose1.pose.position.x, pose1.pose.position.y
    x2, y2 = pose2.pose.position.x, pose2.pose.position.y
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_path_length(path):
    total_distance = 0.0
    previous_pose = None

    for pose_stamped in path.path.poses:
        if previous_pose is not None:
            total_distance += euclidean_distance(previous_pose, pose_stamped)
        previous_pose = pose_stamped

    return total_distance
# Main function
def trajectory_generator():
    
    trajectories = []
 
    if robot_specs['trajectory_type'] == 'user_defined':
        x=robot_specs['user_defined_trajectories'][0]["spawn_pose"]["x"]
        y=robot_specs['user_defined_trajectories'][0]["spawn_pose"]["y"]    
        yaw=robot_specs['user_defined_trajectories'][0]["spawn_pose"]["yaw"]
        initial_pose_=initial_pose(x,y,yaw)
        set_intial_state(initial_pose_,navigator)
        time.sleep(5)
        navigator.clearAllCostmaps()
        navigator.getGlobalCostmap()
        navigator.getLocalCostmap()
        time.sleep(3)
        for traj in robot_specs['user_defined_trajectories']:

            waypoints = process_user_defined_trajectory(traj,robot_specs)
            trajectories.append((waypoints))

    elif robot_specs['trajectory_type'] == 'auto_generated':
        
        i=0
        
        initial_pose_=robot_specs['auto_generated_trajectory']['spawn_pose']
        initial_pose_=initial_pose(initial_pose_['x'],initial_pose_['y'],initial_pose_['yaw'])
        set_intial_state(initial_pose_,navigator)
        
        for traj in robot_specs['auto_generated_trajectory']['types']:
            trajectory_poses,trajectory_type = generate_random_trajectory(traj,navigator,planners_id_list)
            initial_pose_points=[trajectory_poses[0].pose.position.x,trajectory_poses[0].pose.position.y,trajectory_poses[0].pose.orientation.z ,trajectory_poses[0].pose.orientation.w ]
            trajectory_points=[]
            for i in range(1,len(trajectory_poses)):
                trajectory_points.append([trajectory_poses[i].pose.position.x,trajectory_poses[i].pose.position.y,trajectory_poses[i].pose.orientation.z ,trajectory_poses[i].pose.orientation.w])
            trajectories.append((initial_pose_points,trajectory_points,trajectory_type)) 

    # Process the trajectories array as needed
    navigator.destroyNode()
    return trajectories

def main():
    trajectories=trajectory_generator()
    data=[]

    specs = os.environ['PARAMS_FILE']
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
    pdf_name=robot_specs['experiment_name']
    csv_file = os.path.join(get_package_share_directory('ROSNavBench'),
        'raw_data',pdf_name+'_trajectories.csv')
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        # Write header
        writer.writerow(['spawn_x', 'spawn_y', 'spawn_orientation_z', 'spawn_orientation_w','final_x', 'final_y', 'final_orientation_z', 'final_orientation_w', 'traj_type','traj_number'])
        counter=0
        for spawn_pose, final_poses, traj_type in trajectories:

            for final_pose in final_poses:
                row = [*spawn_pose, *final_pose, traj_type,counter]
                writer.writerow(row)
            counter+=1