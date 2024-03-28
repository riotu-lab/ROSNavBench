# Adding a New World

To add a new world for ROSNavBench, follow these steps:

1. **Add the Plugin to the World:**
   - Add the necessary plugin to the world file. This plugin provides essential functionality and interaction within the new world.

```bash
cd /home/ros2_ws/src/ROSNavBench/simulations/worlds/
gedit your_world_name.world
```

copy these lines to your top-level world after `<world name="">`

```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
  <update_rate>1.0</update_rate>
</plugin>
```

2. **Place the World:**

   - Save the world file in the `ROSNavBench/simulations/worlds` directory. This is the designated location for world files used in the ROSNavBench process.

3. **Update Configuration:**

   - In the `ROSNavBench/config` directory, update the configuration file.
   - Add the name of the new world to the configuration. This ensures that the ROSNavBench recognizes the new world.
   - If the world contins models that need to be sourced, add the path to these models to models_path in the configuration file.

> Note: Special case for Turtlebot3

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

    These steps enable the integration of a new world into the ROSNavBench process, ensuring proper functionality and compatibility with the ROSNavBench.
