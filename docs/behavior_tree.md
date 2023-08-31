# Creating a Behavior Tree

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
