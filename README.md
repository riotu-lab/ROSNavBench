# ros2_nav_benchmarking
This repository implements an automated benchamrking methods for any ROS2 compatible navigation framework.

# ROS 2 Navigation Benchmarking

This repository is created for benchmarking navigation in ROS 2. It provides a framework for evaluating the performance and efficiency of navigation systems in ROS 2.

## Getting Started

To get started with benchmarking navigation in ROS 2, follow these steps:

1. Clone this repository: 


```
git clone https://github.com/riotu-lab/ros2_nav_benchmarking
```

2. Install the required dependencies. Make sure you have ROS 2 installed on your system.
```
git clone --branch galatic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
```
```
git clone --branch galatic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

3. Navigate to the cloned repository:

```
cd ros2_nav_benchmarking/src/benchmarking_tool
```

4. Explore the repository structure and its contents. The repository contains the following folders:

- `launch`: Contains launch files for running benchmarking experiments.
- `worlds`: Includes pre-defined world files for simulation environments.

5. Navigate to:
```
cd ros2_nav_benchmarking/src/benchmarking_tool/benchmarking_tool
```
- `benchmarking_tool`: Contains additional scripts and utilities related to benchmarking.

## Usage

Once you have cloned the repository and installed the dependencies, you can start using the benchmarking framework. Here are a few usage examples:

- Launch a benchmarking experiment with a specific configuration:
bash
ros2 launch benchmarking_tool xxx.launch.py


- Modify the launch files in the `launch` folder to customize the benchmarking experiment according to your requirements.

- Explore the pre-defined worlds in the `worlds` folder to simulate different environments.

- Utilize the scripts in the `benchmarking_tool` folder to analyze the benchmarking results or perform additional tasks related to navigation.

## Contributing

We welcome contributions to improve this benchmarking framework. If you have any suggestions, bug reports, or would like to add new features, feel free to create issues or submit pull requests on the [GitHub repository](https://github.com/riotu-lab/ros2_nav_benchmarking).


## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

We would like to acknowledge the contributions of the following individuals to this project:

- [Saqeef Tehnan](https://github.com/saqeeftehnan)
- [Mohamed Abdelkader Zahana](https://github.com/mzahana)
- [Fatimah Alahmed](https://github.com/Fatimah-Alahmed)

Thank you for your support and contributions!
