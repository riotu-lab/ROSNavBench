# Husky Robot

For more information, visit this [repository](https://github.com/khaledgabr77/husky_simulation).

## Required ROS 2 Packages

Install the necessary ROS 2 packages:

```bash
sudo apt-get install ros-humble-hardware-interface ros-humble-controller-manager ros-humble-gazebo-ros2-control ros-humble-xacro ros-humble-hardware-interface ros-humble-gazebo-plugins ros-humble-gazebo-msgs ros-humble-gazebo-ros ros-humble-gazebo-ros2-control-demos ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

## Getting Started

For Ubuntu 22.04, follow these steps:

```bash
cd /<your_workspace>/src
git clone https://github.com/khaledgabr77/husky_simulation
cd ../..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Use the AWS RoboMaker Small Warehouse World, located at `husky_simulation/worlds/industrial_warehouse.world`, along with its model files found in `husky_simulation/models/`.

Copy the world file `industrial_warehouse.world` and the related models from `husky_simulation/models/` to `ROSNavBench/simulations/models/` and `ROSNavBench/simulations/worlds/`.

Also, copy the map files from the `husky_navigation` package, which include `warehouse_slam_toolbox.pgm` and `warehouse_slam_toolbox.yaml`, to `ROSNavBench/simulations/maps/`.

```bash
cp /<your_workspace>/src/husky_simulation/worlds/industrial_warehouse.world /<your_workspace>/src/ROSNavBench/simulations/worlds/
```

```bash
cp /<your_workspace>/src/husky_simulation/models* /<your_workspace>/src/ROSNavBench/simulations/models/
```

```bash
cp /<your_workspace>/src/husky_navigation/maps* /<your_workspace>/src/ROSNavBench/simulations/maps/
```

Next, add the absolute path of your Husky robot URDF, located at `husky_simulation/urdf/husky_ual.urdf.xacro`.

```bash
cd /<your_workspace>/src/ROSNavBench/config/
gedit circle_path_scenario.yaml
```

```yaml
# Add the path to your URDF file
urdf_file: "/<your_workspace>/husky_simulation/urdf/husky_ual.urdf.xacro"
```

For more information, consult the documentation [here](docs/change_robot.md).
