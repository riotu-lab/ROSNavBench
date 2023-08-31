# Trajectories

## Circle

The user can test the control on a circular path.

**Requirements:**

- Specify the initial pose (which will be the center of the circle).

```bash
spawn_pose_x:  -0.5
spawn_pose_y:   0.0
spawn_pose_yaw: 0.0 #radians 
```

- Specify the radius of the circle.

```bash
radius: 0.5
```

## Square

The user can test the control on a square path.

**Requirements:**

- Specify the initial pose (which will be one of the corners of the square).

```bash
spawn_pose_x:    2.5
spawn_pose_y:   -4.25
spawn_pose_yaw: 0.026  #radians 
```

- Specify the side length of the square.

```bash
side_length: 2.5
```

## One_goal

The user can navigate the robot to a single goal pose.
> Note: Please be advised that for the **one_goal** trajectory type, you have the option to choose from the following files: `straight_line.yaml`, `static_obstacles.yaml`, `narrow_path.yaml`, or `dynamic_obstacles.yaml`.

**Requirements:**

- Specify the initial pose.

```bash
spawn_pose_x:   -6.0
spawn_pose_y:    2.0
spawn_pose_yaw: -3.13   #radians 
```

- Specify the goal pose.

```bash
goal_pose_x:  -6.28
goal_pose_y:   1.0
goal_pose_yaw: 1.58   #radians 
```

## Waypoints

The user can guide the robot through multiple poses forming a certain path or visiting specific points.

**Requirements:**

- Specify the initial pose.

```bash
spawn_pose_x:   -6.0
spawn_pose_y:    2.0
spawn_pose_yaw: -3.13   #radians 
```

- Specify the other poses in sequence.

```bash
waypoints: [[-2.4,3.5,-3.13], [6.0,2.0,-3.13], [7.3,2.6,-3.13]]
```

These trajectory types provide users with various options to test and evaluate control strategies using different paths and poses. By specifying the required parameters, users can customize the behavior of the robot accordingly.

## Conﬁguration ﬁle sample

![circle_path_scenario](https://github.com/riotu-lab/ROSNavBench/blob/pr_dev/images/Screenshot%20from%202023-08-29%2017-25-46.png)


| Property                | Description                                                                                         |
|-------------------------|-----------------------------------------------------------------------------------------------------|
| experiment_name        | The name that will appear as the title and the name in the PDF report                             |
| results_directory    | The absolute path of the directory to save the `PDF` report. Use `''` to save results to `ROSNavBench/results`
| world_path             | The absolute path of the world file directory The world file should be located at `ROSNavBench/simulations/worlds`                                      |
| map_path               | The absolute path of the world file directory The world file should be located at `ROSNavBench/simulations/maps`                                              |
| models_path             | If the models are not in the model file of the `ROSNavBench/simulations/models`, add the full path in this property |
| controller_type        | A list of the names of the required controllers. Names should match those in the navigation configuration file. Available controllers are: `DWB`, `DWB_RSC`, `RPP`, `RPP_RSC` |
| trial_num            | This choice is designed for conducting repetitive testing of the same controller. If this testing isn't necessary for the user, it can be disabled by setting it to `zero`.       |                                                 |
| nav_config             | The name of the configuration file for navigation. It should be placed at `ROSNavBench/examples/nav2_config` |
| behaviour_tree_directory | The directory of the behavior trees It should be placed at `ROSNavBench/examples/behavior_trees`                                                              |
| urdf_file               | Absolute path to the URDF file                                                                    |
| model_file              | Absolute path to the model file. If the model file will not be used, set it as `"None"`              |
| spawn_pose_x         | Initial x pose of float type                                                                         |
| spawn_pose_y         | Initial y pose of float type                                                                         |
| spawn_pose_yaw       | Initial yaw angle of float type                                                                      |
| trajectory_type      | There are four types of possible trajectories: `one_goal`, `waypoints`, `square`, and `circle`.     |
| goal_pose_x           | Goal x pose of float type. Fill if the trajectory type is `one_goal`.                                |
| goal_pose_y           | Goal y pose of float type. Fill if the trajectory type is `one_goal`.                                |
| goal_pose_yaw         | Goal yaw pose of float type. Fill if the trajectory type is `one_goal`.                              |
| radius                | Radius of the circle. Fill if the trajectory type is `circle`.                                       |
| side_length           | Side length of the square. Fill if the trajectory type is `square`.                                 |
| waypoints             | A list where each element is `[x, y, yaw]`. The numbers should be floats. Fill if trajectory type is `waypoints`. |



Mandatory to fill. Others properties are dependent on the case.