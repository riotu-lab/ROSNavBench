#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class PoseGoalPublisher(Node):
    def __init__(self):
        super().__init__("pose_goal_publisher")

        # Create publishers for goal
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)

        # Create subscriber to get the odom feedback
        # self.pose_sub = self.create_subscription(Pose, '/odom', self.pose_callback, 10)

        # Set up a timer to publish messages periodically
        self.create_timer(1.0, self.publish_messages)


    # def pose_callback(self,msg):
    #     self.x = msg.x
    #     self.y = msg.y
    #     self.yaw = msg.theta
    
    
    def publish_messages(self):

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 0.59
        goal.pose.position.y = -0.53
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        while rclpy.ok():
                self.get_logger().info("Turtlebot is moving...")
                self.goal_pub.publish(goal)
                rclpy.spin_once(self)

                # current_position = self.get_current_position()
                # distance_to_goal = ((current_position.pose.position.x - goal.pose.position.x) ** 2 + (current_position.pose.position.y - goal.pose.position.y) ** 2) ** 0.5
                # self.get_logger().info(f"Distance to goal: {distance_to_goal:.2f}")
                # if distance_to_goal < 0.1:
                #     self.node.get_logger().info("Goal reached")
                #     break

                # self.get_logger().info("Turtlesim reached the desired position")
                #   

        #finally, stop the robot when the distance is moved        
        goal.linear.x = 0.0
        self.goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    go_to_goal = PoseGoalPublisher()
    rclpy.spin(go_to_goal)
    go_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
