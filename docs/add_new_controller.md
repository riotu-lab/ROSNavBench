# Adding a New Controller or Planner

To add a new controller for ROSNavBench, follow these steps:

2. **Add a Controller the Nav2 Configuration File:**

2. **Update the Main Configuration File:**
   - In the main configuration file located in `ROSNavBench/config`, add the name of the new controller to the `controller_type` property. This ensures that the ROSNavBench recognizes and operates using the new controller.

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
