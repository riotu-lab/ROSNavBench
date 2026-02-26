# ROSNavBench
ROSNavBench is an open-source framework designed to facilitate the testing and comparison of various local and global planners in the Robot Operating System (ROS) 2. This tool provides a comprehensive platform for evaluating planner performance across diverse trajectories, collecting several metrics such as navigation time, path length, and success rate.

# Overview
Efficient autonomous navigation in mobile robotics demands robust path planning capabilities. Despite the availability of numerous evaluation tools and frameworks for path planners, there is a significant gap in open-source, automated frameworks specifically tailored for ROS 2. ROSNavBench addresses this gap by providing a comprehensive statistical analysis necessary for robust decision-making. This framework automates the benchmarking of ROS 2-based local and global path planners, offering quantitative assessments across critical metrics such as efficiency, reliability, and adaptability under diverse environmental scenarios.

# Key Features
- **Comprehensive Testing**: Perform tests and comparisons among diverse local and global planners across various types of trajectories.
- **Quantitative Evaluation**: Assess the efficiency of planners using various metrics such as safety, success rate, path length, navigation time, path deviation, and number of recoveries.
- **Automated Iterative Testing**: Enhance result reliability through statistical analysis and streamline the workflow.
- **Gazebo Sim Integration**: Built on Gazebo Sim (GZ) for modern physics simulation and improved performance.

# Prerequisites

Before using the ROS 2 `ROSNavBench`, make sure you have the following prerequisites in place:

- **ROS 2 Installation**: Ensure you have ROS 2 [`Humble`](docs/install_humble.md) installed on your system. This forms the foundation for running the `ROSNavBench`.

- **Python Libraries**: Install the following Python libraries to enable the functionality of the `ROSNavBench`:

  - **Csv**: Used for handling CSV file formats.

    ```bash
    pip3 install python-csv
    ```

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

    ```bash
    sudo apt install ros-humble-nav2-simple-commander
    ```

- **Gazebo Sim**: ROSNavBench utilizes Gazebo Sim (GZ) for physics simulation. Install the required Gazebo Sim packages:

    ```bash
    sudo apt install \
      ros-$ROS_DISTRO-ros-gz-sim \
      ros-$ROS_DISTRO-ros-gz-bridge \
      ros-$ROS_DISTRO-ros-gz-image
    ```

- **Turtlebot3**: The ROSNavBench package employs the `turtlebot3` robot as the default choice for evaluating its functionalities:

    ```bash
    sudo apt install ros-$ROS_DISTRO-turtlebot3*
    ```

- **Nav2**: Install the Nav2 packages using your operating system's package manager:

    ```bash
    sudo apt install \
      ros-$ROS_DISTRO-navigation2 \
      ros-$ROS_DISTRO-nav2-bringup 
    ```

Please ensure that these prerequisites are satisfied before proceeding with the `ROSNavBench` setup and usage.

## Installation

1. **Clone the Repository**:

    ```bash
    git clone https://github.com/riotu-lab/ROSNavBench
    ```

2. **Build the Workspace**:

    Navigate to your ROS 2 workspace and build the package:

    ```bash
    cd ~/your_workspace/src
    colcon build --packages-select ROSNavBench
    source install/setup.bash
    ```

## Recent Updates

### Migration to Gazebo Sim

ROSNavBench has been migrated from Gazebo Classic to **Gazebo Sim (GZ)**, providing improved simulation performance and compatibility with modern ROS 2 distributions. Key improvements include:

- **Enhanced Simulation Performance**: Leverages Gazebo Sim's improved physics engine and rendering capabilities
- **Modern ROS 2 Integration**: Utilizes `ros_gz_sim`, `ros_gz_bridge`, and `ros_gz_image` packages for seamless integration
- **Updated Robot Models**: Includes Gazebo Sim-compatible robot models, world files, and URDF configurations
- **Improved Launch System**: Refactored launch files for better parameter handling and spawn pose management
- **Navigation Enhancements**: Updated Nav2 configurations and behavior tree templates for optimal performance

The framework now includes Gazebo Sim-compatible simulation files located in the `simulations/` directory, including world files (`.sdf`), robot models, and URDF configurations specifically designed for Gazebo Sim.

## Launching ROSNavBench Tool

To initiate the ROSNavBench test, follow these steps:

1. **Export File Name and Run**:
    - You can pass the configuration file directly with the `params_file` launch argument (recommended), or set `PARAMS_FILE` for backward compatibility.
      - Example using a relative path (resolved relative to the package):

      ```bash
      ros2 launch ROSNavBench main.launch.py params_file:=config/house_experiment_no_obstaclesyaml.yaml
      ```

      - Example using an absolute path:

      ```bash
      ros2 launch ROSNavBench main.launch.py params_file:=/home/USER/riout_ws/src/ROSNavBench/config/house_experiment_no_obstaclesyaml.yaml
      ```

      - Example using `PARAMS_FILE`:

      ```bash
      export PARAMS_FILE="/home/USER/riout_ws/src/ROSNavBench/config/house_experiment_no_obstaclesyaml.yaml"
      ros2 launch ROSNavBench main.launch.py
      ```

2. **Accessing Results**:
   - Once the execution of all the tests is complete, the generated files will be automatically saved to the specified results folder.
   - You can find the raw results in the `share` directory of the ``ROSNavBench`` package.

> **Note**: More information about the configuration check [here](docs/Trajectories.md).

By following these steps, you'll be able to execute the benchmarking test.

## Configuration Notes

- **Relative paths** in `config/*.yaml` are resolved relative to the config file location (e.g., `world_path`, `nav_config`, `urdf_file`, `behaviour_tree_directory`).
- **results_directory**:
  - If absolute, it is used as-is.
  - If relative (e.g., `results`), it is resolved to the ROSNavBench package root and created if missing.
- **models_path** supports multiple entries separated by `:` and each entry can be relative.

## Built-in Controllers and Planners

- **Controllers**: `DWB`, `RPP`, `DWB_RSC`, `MPPI`
- **Planners**: `GridBased` (NavFn), `NavFn`, `smac_planner`, `ThetaStar`, `Lattice`, `SmacHybraid`

## Launching a custom Example

To test with your custom robots, for example husky robot, follow these steps:

1. **Specify the robot**
   - Follow the instructions inside the documentation to specfiy a robot. check [here](docs/change_robot.md)

2. **Update your World**
    - To add a new world for ROSNavBench, follow the instructions [here](docs/add_new_world.md)
    - **Note**: World files should be in Gazebo Sim SDF format (`.sdf`) for compatibility with the current version.

3. **Configure Example Settings**:
   - Open the configuration file located at `ROSNavBench/config`.
   - Edit the absolute paths and other parameters to match your machine's setup such as the `world`, `map`, `controllers`, and `urdf_file`...
   - Customize the parameters according to the example you want to run. check [here](docs/Trajectories.md)

4. **Adding a New Controller or Planner**
    - To add a new controller or planner for `ROSNavBench`, follow the instructions [here](docs/add_new_controller.md)

5. **Adding a New Behavior Tree**
    - To add a new Behavior Tree(BT) for `ROSNavBench`, follow the instructions [here](docs/behavior_tree.md)

6. **Build the Workspace**:
   - In your terminal, build the workspace where the `ROSNavBench` is located.

    ```bash
    colcon build
    source install/setup.bash
    ```

7. **Set the Parameters File and Launch**:
   - Launch using the `params_file` argument (or `PARAMS_FILE` for backward compatibility).
      - For example:

      ```bash
      ros2 launch ROSNavBench main.launch.py params_file:=config/house_experiment_no_obstaclesyaml.yaml
      ```

   - Or:

      ```bash
      export PARAMS_FILE="/home/USER/riout_ws/src/ROSNavBench/config/house_experiment_no_obstaclesyaml.yaml"
      ros2 launch ROSNavBench main.launch.py
      ```

8. **Accessing Results**:
   - Once the execution of all the tests is complete, the generated files will be automatically saved to the specified results folder.
   - You can find the raw results in the `share` directory of the ``ROSNavBench`` package.

By following these steps, you'll be able to effortlessly launch and execute the provided example, view the results, and adapt the tool to your specific machine configuration.
