import pandas as pd

from ament_index_python.packages import get_package_share_directory
import os

def extract_data(file_path,trajectory_num):
    # Load the CSV file
    trajectories_df = pd.read_csv(file_path)
    # Filtering the dataframe for a specfic trajectories
    traj= trajectories_df[trajectories_df['traj_number'] == trajectory_num]
    indices = traj.index 
    print(indices[0])
    initial_pose=[traj.loc[indices[0], 'spawn_x'],traj.loc[indices[0], 'spawn_y'],traj.loc[indices[0], 'spawn_orientation_z'],traj.loc[indices[0], 'spawn_orientation_w']]
    trajectory_type=traj.loc[indices[0],'traj_type']
    trajectory_points=[]
    for i in range(traj.shape[0]):
        trajectory_points.append([traj.loc[indices[i], 'final_x'],traj.loc[indices[i], 'final_y'],traj.loc[indices[i], 'final_orientation_z'],traj.loc[indices[i], 'final_orientation_w']])

    return trajectory_type, initial_pose,trajectory_points

