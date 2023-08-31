# Changing the Robot

When changing the robot used in the benchmarking process, follow these steps:

1. **URDF and Model Files:**
   - The robot is spawned using a robot state publisher and Gazebo's spawn entity.
   - Define the URDF file and model file of the new robot.
   - If the robot is spawned only through a URDF file, mark the model as `"None"`.

      ```yaml
      model_file: "None"
      ```

2. **Additional Nodes:**
   - If you require additional nodes to support the new robot's functionality, add these nodes to the `ROSNavBench/launch/spawn_robot.launch.py` file.
   - Ensure that the nodes are integrated correctly into the launch process to facilitate proper robot operation.

By following these steps, you can effectively change the robot used in the benchmarking process and ensure that the necessary URDF, model files, and additional nodes are properly configured for the new robot.
