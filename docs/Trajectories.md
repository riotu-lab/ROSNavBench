# Trajectories
The user can specify several trajectories to be tested. These trajectories can be either "user_defined", "auto_generated"
```bash
trajectory_type: "user_defined"
```
## User-defined trajectories 
The options are a single goal pose, multiple waypoints, circle, and square.  Specify as much trajectories as you wish as shown below. 
```bash
user_defined_trajectories:
  - type: "single_goal"
    spawn_pose:
      x: 6.30
      y: 1.26
      yaw: 0.0
    single_goal:
      x: -5.6
      y: -5.6
      theta: 0.0
  - type: "waypoints"
    spawn_pose:
      x: 4.97
      y: 1.12
      yaw: 0.0
    waypoints:
      - x: -0.01
        y: 2.0
        theta: 0.0
      - x: -6.5
        y: 6.37
        theta: 0.0        
      - x: -5.0
        y: -3.0
        theta: 0.0
      - x: 6.37
        y: -6.41
        theta: 1.57
  - type: "circle"
    circle_center:
      x: -2.5
      y: 5.48
    circle:
      radius: 1.3
  - type: "square"
    spawn_pose:
      x: -3.5
      y: 4.3
      yaw: 0.0
    square:
      side_length: 2.7

```

### Circle

The user can test the controller and planner on a circular path.

**Requirements:**
- Specify the type as "circle".
- Specify the center of the circle
- Specify the radius of the circle.

```bash
  - type: "circle"
    circle_center:
      x: -2.5
      y: 5.48
    circle:
      radius: 1.3
```

### Square

The user can test the controller and planner on a square path.

**Requirements:**

- Specify the initial pose (which will be one of the corners of the square).
- Specify the side length of the square.

```bash
  - type: "square"
    spawn_pose:
      x: -3.5
      y: 4.3
      yaw: 0.0
    square:
      side_length: 2.7
```

### Single goal 

The user can navigate the robot to a single goal pose.
> Note: Please be advised that for the **one_goal** trajectory type, you have the option to choose from the following files: `straight_line.yaml`, `static_obstacles.yaml`, `narrow_path.yaml`, or `dynamic_obstacles.yaml`.

**Requirements:**
- Specify the type as "single_goal".
- Specify the initial pose.
- Specify the goal pose.

```bash
  - type: "single_goal"
    spawn_pose:
      x: 6.30
      y: 1.26
      yaw: 0.0
    single_goal:
      x: -5.6
      y: -5.6
      theta: 0.0
```


### Waypoints

The user can guide the robot through multiple poses forming a certain path or visiting specific points.

**Requirements:**
- Specify the type as "single_goal".
- Specify the initial pose.
- Specify the other poses in sequence.

```bash
  - type: "waypoints"
    spawn_pose:
      x: 4.97
      y: 1.12
      yaw: 0.0
    waypoints:
      - x: -0.01          # First point
        y: 2.0
        theta: 0.0
      - x: -6.5           # Secondpoint
        y: 6.37
        theta: 0.0        
      - x: -5.0           # Third point 
        y: -3.0
        theta: 0.0
      - x: 6.37           # Fourth point
        y: -6.41
        theta: 1.57
```


## Auto generated trajectories 
The options are: short_trajectory, long_trajectory, waypoints.  Specify as many trajectories as you wish as shown below. Note that a single spawn pose is needed; however, this will not be the spawn pose for the trajectories. 
```bash
auto_generated_trajectory:
  spawn_pose:
    x: 4.0
    y: -3.0
    yaw: -3.13
  types:
    - type: "short_trajectory"
      short_trajectory:
        upper_bound: 7
    - type: "long_trajectory"
      long_trajectory:
        lower_bound: 8
    - type: "waypoints"
      waypoints: 5
```     
### Short trajectory
Specify the path by giving an upper limit that the path length should not exceed.
### Long trajectory
Specify the path by giving a lower limit that the path length should be longer.
### Waypoints 
Specify the path by indicating the number of required waypoints. 


These trajectory types provide users with various options to test and evaluate local and global path planner strategies using different paths and poses. By specifying the required parameters, users can customize the behavior of the robot accordingly.

## Conﬁguration ﬁle sample

![circle_path_scenario](https://github.com/riotu-lab/ROSNavBench/blob/Multi_traj_version/images/configuration_file_example.png)


| Property                | Description                                                                                         |
|-------------------------|-----------------------------------------------------------------------------------------------------|
| experiment_name        | The name that will appear as the title and the name in the PDF report                             |
| results_directory    | The absolute path of the directory to save the `PDF` report. Use `''` to save results to `ROSNavBench/results`
| world_path             | The absolute path of the world file directory                                                     |
| map_path               | The absolute path of the map yaml file                                                            |
| map_png_path           | The absolute path of the map png file                                                             |
| models_path             | If the models are not in the model file of the `ROSNavBench/simulations/models`, add the full path in this property. |
| controller_type        | A list of the names of the required controllers. Names should match those in the navigation configuration file. Available controllers are: `DWB`, `DWB_RSC`, `RPP`, `RPP_RSC`, 'MPPI' |
| planner_type        | A list of the names of the required planners. Names should match those in the navigation configuration file. Available planners are: "NavFn", "smac_planner", "ThetaStar", "Lattice" |
| instances_num            | This choice is designed for conducting repetitive testing of the same controller. If this testing isn't necessary for the user, set it to 1.       |                                                 |
| nav_config             | The absolute path of the configuration file for navigation.                                                  |
| behaviour_tree_directory | The absolute path of the directory of the behavior trees                                                       |
| urdf_file               | Absolute path to the URDF file                                                                    |
| model_file              | Absolute path to the model file. If the model file will not be used, set it as `"None"`              |
| trajectory_type         | "user_defined", "auto_generated"                                                                        |
| criteria                | Citeria for performace analysis includes: "Time", "CPU", "Memory", "path_length", "Safety", 'path_deviation', 'success_rate', 'number_of_recoveries'.                                                                 |
| weights                 | Weight for each criterion as a number from 1 to 9 arranged into a matrix. Set None if the weight to be set automatically giving that importance to the earlier criteria in the matrix.                                                                       |


Mandatory to fill. Other properties are dependent on the case.
