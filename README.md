# ROSNavBench

The **`ROSNavBench`** empowers users to perform testing and comparisons between various local and global planners across various trajectories. 

## Introduction

The **`ROSNavBench`** provides a comprehensive platform for evaluating the performance of different local and global planners across various trajectories. Several metrics are collected through out the test such as navigation time. 

## Key Features

- Perform tests and comparisons among diverse local and global planners acrros various types of trajectories.
- Evaluate efficiency of the global and local planners through various metrics. 

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
    - In your terminal, export the absolute path of the configuration file using the appropriate command. Make sure that the file is configured to your machine.
      - For example:

      ```bash
      export PARAMS_FILE="\home\Absolut_path\configuration_file.yaml"
      ```

    - Run the `main` script to start the test execution.

      ```bash
      ROS2 launch ROSNavBench main.launch.py
      ```

2. **Accessing Results**:
   - Once the execution of all the tests is complete, the generated files will be automatically saved to the specified results folder.
   - You can find the raw results in the `share` directory of the ``ROSNavBench`` package.

> **Note**: More information about the configuration check [here](docs/Trajectories.md).

By following these steps, you'll be able to execute the benchmarking test.

## Launching a custom Example

To test with your custom robots, for example husky robot, follow these steps:

1. **Specify the robot**
   - Follow the instructions inside the documentation to specfiy a robot. check [here](docs/change_robot.md)

2. **Update your World**
    - To add a new world for ROSNavBench, follow the instructions [here](docs/add_new_world.md)
 
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
   - Set the `PARAMS_FILE` variable to the desired example configuration and launch the test`
      - For example:

      ```bash
      export PARAMS_FILE="\home\Absolut_path\configuration_file.yaml"
      ```

    - Run the `main` script to start the test execution.

      ```bash
      ROS2 launch ROSNavBench main.launch.py
      ```

8. **Accessing Results**:
   - Once the execution of all the tests is complete, the generated files will be automatically saved to the specified results folder.
   - You can find the raw results in the `share` directory of the ``ROSNavBench`` package.

By following these steps, you'll be able to effortlessly launch and execute the provided example, view the results, and adapt the tool to your specific machine configuration.
