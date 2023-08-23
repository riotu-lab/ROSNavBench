# Benchmarking Tool

The **Benchmarking Tool** empowers users to perform testing and comparisons between various local planners. It also facilitates benchmarking of an individual local planner.

## Introduction

The **Benchmarking Tool** provides a comprehensive platform for evaluating the performance of different local planners. It allows users to compare the efficiency and effectiveness of various options. Additionally, the tool facilitates benchmarking exercises for a single local planner, helping users gain insights into its performance metrics.

## Key Features

- Perform tests and comparisons among diverse local planners.
- Facilitate benchmarking of individual local planners.
- Evaluate efficiency and effectiveness of local planning strategies.

## Prerequisites

Before using the ROS2 Benchmarking Tool, make sure you have the following prerequisites in place:

- **ROS2 Installation**: Ensure you have ROS2 `Humble` installed on your system. This forms the foundation for running the Benchmarking Tool.

- **Python Libraries**: Install the following Python libraries to enable the functionality of the Benchmarking Tool:

  - **Csv**: Used for handling CSV file formats.
  - **Reportlab**: Required for generating PDF reports.

    ```bash
    pip3 install reportlab
    ```

  - **Jinja2**: This library is used for template rendering.

    ```bash
    pip3 install Jinja2
    ```

  - **psutil**: Enables monitoring of system resources.

    ```bash
    pip3 install psutil
    ```

  - **Nav2 Simple Commander**: Essential for navigation tasks and commands.

Please ensure that these prerequisites are satisfied before proceeding with the Benchmarking Tool setup and usage.

## Launching the Test

To initiate the benchmarking test, follow these steps:

1. **Fill the Configuration File**:
    - Open the configuration file that defines essential parameters such as the `world`, `map`, `controllers`, and `scenario`.
    - Customize the settings according to your test requirements.

2. **Export File Name and Run**:
    - In your terminal, export the file name of the configuration file using the appropriate command.
    - Run the `main` script to start the test execution.

**Note**: The configuration file should be placed within the `benchmarking_tool/config` directory.

By following these steps, you'll be able to execute the benchmarking test with the specified configuration settings.

## Execution and Export

To execute the benchmarking test and export the results, follow these steps:

1. **Set the Parameters File**:
   - In your terminal, set the `PARAMS_FILE` variable to the name of the configuration file you filled earlier.
   - For example: `PARAMS_FILE="name_of_config_file"`

2. **Run the Launch Command**:
   - Launch the benchmarking test using the following ROS 2 launch command:

     ```
     ros2 launch benchmarking_tool main.launch.py
     ```

3. **Accessing Results**:
   - Once the execution of the test goals is complete, the generated files will be automatically saved to the `results` folder within the `benchmarking_tool` directory.
   - You can find the results in the `share` directory of the `benchmarking_tool` package.

By following these steps, you'll be able to execute the test, gather results, and access them for analysis and reporting.

## Launching a Ready Example

To quickly launch a pre-configured example, follow these steps:

1. **Configure Example Settings**:
   - Open the configuration file located at `benchmarking_tool/config`.
   - Edit the absolute paths and other parameters to match your machine's setup.
   - Customize the parameters according to the example you want to run.

2. **Source the Workspace**:
   - In your terminal, source the workspace where the benchmarking tool is located.

3. **Set the Parameters File and Launch**:
   - Set the `PARAMS_FILE` variable to the desired example configuration, e.g.: `PARAMS_FILE=static_obstacles`
   - Launch the benchmarking test using the ROS 2 launch command:

     ```
     ros2 launch benchmarking_tool main.launch.py
     ```

4. **Accessing Results**:
   - Once the test execution is completed, you can find the results in the `results` folder within the `benchmarking_tool` directory.
   - The results will be stored in the `share` directory or in the specified directory based on your configuration.

By following these steps, you'll be able to effortlessly launch and execute the provided example, view the results, and adapt the tool to your specific machine configuration.

### Conﬁguration ﬁle sample

![circle_path_scenario](https://github.com/riotu-lab/ROSNavBench/blob/pr_husky/images/Screenshot%20from%202023-08-23%2019-57-14.png)


| Property                | Description                                                                                         |
|-------------------------|-----------------------------------------------------------------------------------------------------|
| experiment_name        | The name that will appear as the title and the name in the PDF report                             |
| world_name             | The world file should be located at `benchmarking_tool/world`                                      |
| map_name               | The map should be located at `benchmarking_tool/map`                                              |
| models_path             | If the models are not in the model file of the `benchmarking_tool`, add the full path in this property |
| controller_type        | A list of the names of the required controllers. Names should match those in the navigation configuration file. Available controllers are: DWB, DWB_RSC, RPP, RPP_RSC |
| nav_config             | The name of the configuration file for navigation. It should be placed at `turtlebot3/turtlebot3_navigation2/param` |
| behaviour_tree_directory | The directory of the behavior trees                                                              |
| urdf_file               | Absolute path to the URDF file                                                                    |
| model_file              | Absolute path to the model file. If the model file will not be used, set it as 'None'              |
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
| trial_num            | `0` for not benchmarking a single controller.                                                        |
| results_directory    | The absolute path of the directory to save the PDF report. Use `''` to save results to `benchmarking_tool/results 

Mandatory to fill. Others properties are dependent on the case.

## Trajectories

### Circle

The user can test the control on a circular path.

**Requirements:**
- Specify the initial pose (which will be the center of the circle).
- Specify the radius of the circle.

## Square

The user can test the control on a square path.

**Requirements:**
- Specify the initial pose (which will be one of the corners of the square).
- Specify the side length of the square.

## One_goal

The user can navigate the robot to a single goal pose.

**Requirements:**
- Specify the initial pose.
- Specify the goal pose.

## Waypoints

The user can guide the robot through multiple poses forming a certain path or visiting specific points.

**Requirements:**
- Specify the initial pose.
- Specify the other poses in sequence.

These trajectory types provide users with various options to test and evaluate control strategies using different paths and poses. By specifying the required parameters, users can customize the behavior of the robot accordingly.

### Adding a New World

To add a new world for benchmarking, follow these steps:

1. **Add the Plugin to the World:**
   - Add the necessary plugin to the world file. This plugin provides essential functionality and interaction within the new world.

2. **Place the World:**
   - Save the world file in the `benchmarking_tool/world` directory. This is the designated location for world files used in the benchmarking process.

3. **Update Configuration:**
   - In the `benchmarking_tool/config` directory, update the configuration file.
   - Add the name of the new world to the configuration. This ensures that the benchmarking tool recognizes the new world.

4. **Adjust Physics Tag for Turtlebot3:**
   - If the new world is intended for use with the Turtlebot3 robot, make an additional adjustment.
   - Locate the `<physics>` tag in the world file and modify it to match the following configuration. This ensures that the Turtlebot3 moves correctly without unexpected rotation issues.

   ```xml
   <physics name="default_physics" default="true" type="ode">
     <max_step_size>0.001</max_step_size>
     <real_time_update_rate>1000</real_time_update_rate>
   </physics>
   <!-- Replace existing <physics> tag with the following -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <update_rate>1.0</update_rate>
    </plugin>
    ```

    These steps enable the integration of a new world into the benchmarking process, ensuring proper functionality and compatibility with the benchmarking tool.

## Adding a New Controller

To add a new controller for benchmarking, follow these steps:

1. **Create a Behavior Tree:**
   - Create a new behavior tree where you specify the controller using the same name as the configuration file. This behavior tree defines the control logic for the new controller.

2. **Update the Main Configuration File:**
   - In the main configuration file located in `benchmarking_tool/config`, add the name of the new controller to the `controller_type` property. This ensures that the benchmarking tool recognizes and operates using the new controller.

By following these steps, you can seamlessly introduce a new controller into the benchmarking process. Ensure that the behavior tree and configuration files are consistent with the intended control logic.

## Adding a Controller to nav2 Configuration File

To add a new controller to the nav2 configuration file, follow these steps:

1. **Add Controller Name to List:**
   - Locate the line containing the `controller_plugins` list within the nav2 configuration file.
   - Add the name of the new controller to the list. The names should match the names mentioned in the provided list.

   ```yaml
   controller_plugins: ["DWB_RSC", "RPP", "DWB", "RPP_RSC", "New_Controller_Name"]
   ```

2. **Add Controller Parameters:**
   - Under the same name as mentioned in the list above (e.g., "New_Controller_Name"), add the parameters for the new controller.
   - Define the plugin, primary_controller, and any other relevant parameters.

   ```yaml
   New_Controller_Name:
     plugin: "your_plugin_namespace::YourControllerPlugin"
     primary_controller: "primary_controller_namespace::PrimaryControllerPlugin"
     debug_trajectory_details: True
     angular_dist_threshold: 0.785
     forward_sampling_distance: 0.5
     rotate_to_heading_angular_vel: 1.8
     max_angular_accel: 3.2
     simulate_ahead_time: 1.0
   ```

These steps ensure the proper integration of the new controller into the nav2 configuration. Make sure to replace "New_Controller_Name", "your_plugin_namespace::YourControllerPlugin", and "primary_controller_namespace::PrimaryControllerPlugin" with the appropriate values based on your new controller.


## Creating a Behavior Tree

To create a behavior tree for a new controller, follow these steps:

1. **Define Behavior Tree:**
   - Create a behavior tree that adheres to your specific requirements.
   - The key requirement is to set the name of the controller as the one mentioned earlier.

   Example behavior tree snippet:

   ```xml
   <RecoveryNode number_of_retries="1" name="New_Controller_Name">
       <FollowPath path="{path}" controller_id="New_Controller_Name"/>
       <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
   </RecoveryNode>
   ```

2. **Create Two Behavior Trees:**
   - Create two behavior trees: one for navigation through poses and the other for navigation to a pose.
   - The naming convention for these files should be `controllerName_poses.xml` and `controllerName.xml` respectively.
   - For example, if the controller name is RPP, you would have `RPP_poses.xml` and `RPP.xml` behavior tree files.
   - Place all the behavior tree files in a single folder.

By following these steps, you can create behavior trees tailored to your new controller's functionality. Make sure to customize the behavior trees based on the control logic and requirements of your specific controller.

## Changing the Robot

When changing the robot used in the benchmarking process, follow these steps:

1. **URDF and Model Files:**
   - The robot is spawned using a robot state publisher and Gazebo's spawn entity.
   - Define the URDF file and model file of the new robot.
   - If the robot is spawned only through a URDF file, mark the model as "None".

2. **Additional Nodes:**
   - If you require additional nodes to support the new robot's functionality, add these nodes to the `benchmarking_tool/launch/spawn_robot.launch.py` file.
   - Ensure that the nodes are integrated correctly into the launch process to facilitate proper robot operation.

By following these steps, you can effectively change the robot used in the benchmarking process and ensure that the necessary URDF, model files, and additional nodes are properly configured for the new robot.
