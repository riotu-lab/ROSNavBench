# ROSNavBench

The **`ROSNavBench`** empowers users to perform testing and comparisons between various local planners. It also facilitates benchmarking of an individual local planner.

## Introduction

The **`ROSNavBench`** provides a comprehensive platform for evaluating the performance of different local planners. It allows users to compare the efficiency and effectiveness of various options. Additionally, the tool facilitates benchmarking exercises for a single local planner, helping users gain insights into its performance metrics.

## Key Features

- Perform tests and comparisons among diverse local planners.
- Facilitate benchmarking of individual local planners.
- Evaluate efficiency and effectiveness of local planning strategies.

## Prerequisites

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

- **Turtelbot3**: The ROSNavBench package employs the `turtlebot3` robot as the default choice for evaluating its functionalities:

    ```bash
    sudo apt install ros-$ROS_DISTRO-turtlebot3*
    ```

    ```bash
    export TURTLEBOT3_MODEL=waffle
    ```

    ```bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
    ```

- **Nav2**: Install the Nav2 packages using your operating system’s package manager:

    ```bash
    sudo apt install \
      ros-$ROS_DISTRO-navigation2 \
      ros-$ROS_DISTRO-nav2-bringup 
    ```

Please ensure that these prerequisites are satisfied before proceeding with the `ROSNavBench` setup and usage.

## Installation

Git clone `ROSNavBench` tool

```bash
git clone https://github.com/riotu-lab/ROSNavBench
```

## Launching ROSNavBench Tool

To initiate the ROSNavBench test, follow these steps:

1. **Export File Name and Run**:
    - In your terminal, export the file name of the configuration file using the appropriate command.
      - For example:

      ```bash
      export PARAMS_FILE="circle_path_scenario"
      ```

    - Run the `main` script to start the test execution.

      ```bash
      ROS2 launch ROSNavBench main.launch.py
      ```

2. **Accessing Results**:
   - Once the execution of the test goals is complete, the generated files will be automatically saved to the `results` folder within the ``ROSNavBench`` directory.
   - You can find the results in the `share` directory of the ``ROSNavBench`` package.

> **Note**: The configuration file should be placed within the `ROSNavBench/config` directory. more information about the configuration check [here](docs/Trajectories.md).

By following these steps, you'll be able to execute the benchmarking test with the `defualt configuration settings`. also, gather results and access them for analysis and reporting.

## Launching a custom Example

To test with your custom robots, for example husky robot, follow these steps:

1. **Install Husky**
   - Follow the instructions inside the documentation to install husky robot. check [here](docs/husky_setup.md)

2. **Update your World**
    - To add a new world for ROSNavBench, follow the instructions [here](docs/add_new_world.md)
 
3. **Configure Example Settings**:
   - Open the configuration file located at `ROSNavBench/config`.
   - Edit the absolute paths and other parameters to match your machine's setup such as the `world`, `map`, `controllers`, and `urdf_file`...

   - Customize the parameters according to the example you want to run. check [here](docs/Trajectories.md)


4. **Adding a New Controller**
    - To add a new controller for `ROSNavBench`, follow the instructions [here](docs/add_new_controller.md)

5. **Adding a New Behavior Tree**
    - To add a new Behavior Tree(BT) for `ROSNavBench`, follow the instructions [here](docs/behavior_tree.md)

6. **Build the Workspace**:
   - In your terminal, build the workspace where the `ROSNavBench` is located.

    ```bash
    colcon build
    source install/setup.bash
    ```

7. **Set the Parameters File and Launch**:
   - Set the `PARAMS_FILE` variable to the desired example configuration, e.g.: `PARAMS_FILE=static_obstacles`

      ```bash
      export PARAMS_FILE="circle_path_scenario"
      ```

   - Launch the benchmarking test using the ROS 2 launch command:

     ```bash
     ROS2 launch ROSNavBench main.launch.py
     ```

8. **Accessing Results**:
   - Once the test execution is completed, you can find the results in the `results` folder within the ``ROSNavBench`` directory.
   - The results will be stored in the `share` directory or in the specified directory based on your configuration.

By following these steps, you'll be able to effortlessly launch and execute the provided example, view the results, and adapt the tool to your specific machine configuration.
