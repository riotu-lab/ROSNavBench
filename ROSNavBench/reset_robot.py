import subprocess
import numpy as np
import json
import os
from ROSNavBench.extract_data import extract_data
from ament_index_python.packages import get_package_share_directory
import yaml


def main():
    specs = os.environ['PARAMS_FILE']
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
    pdf_name=robot_specs['experiment_name']
    trajectory_num=os.environ["trajectory_num"]
    # Example values for x, y, yaw
    csv_path=os.path.join(get_package_share_directory('ROSNavBench'),
        'raw_data',pdf_name+'_trajectories.csv')
    trajectory_type, initial_pose,trajectory_points=extract_data(csv_path,int(trajectory_num))
    # Construct the data as a Python dictionary
    data = {
        "state": {
            "name": "robot",
            "pose": {
                "position": {"x": initial_pose[0], "y": initial_pose[1], "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": initial_pose[2], "w": initial_pose[3]}
            },
            "reference_frame": "world"
        }
    }

    # Convert dictionary to JSON string
    json_data = json.dumps(data)

    command = [
        'ros2', 'service', 'call',
        '/set_entity_state',
        'gazebo_msgs/SetEntityState',
        json_data
    ]
    try_=True
    while try_:
        # Execute the command
        result = subprocess.run(command, capture_output=True, text=True)

        # Check result
        if result.returncode == 0:
            print("Command executed successfully")
            try_=False
        else:
            print("Error in executing command:", result.stderr)
        
