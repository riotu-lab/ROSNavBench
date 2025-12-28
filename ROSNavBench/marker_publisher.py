#! /usr/bin/env python

import rclpy
from visualization_msgs.msg import Marker
import visualization_msgs
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
import os 

class Marker_Publisher(Node):

    def __init__(self):
        super().__init__('rviz_marker')
        self.declare_parameter('planner', '')
        self.declare_parameter('controller', '')
        self.declare_parameter('round_num', '')
        self.planner = self._get_value('planner', 'planner')
        self.controller = self._get_value('controller', 'controller')
        self.round_num = self._get_value('round_num', 'round_num')
        self.publisher_ = self.create_publisher(Marker, "/visualization_marker", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def _get_value(self, param_name, env_name):
        value = self.get_parameter(param_name).get_parameter_value().string_value
        if value:
            return value
        return os.environ.get(env_name, '')

    def timer_callback(self):
        self.publisher_.publish(marker)



def main(args=None):
    rclpy.init(args=args)

    marker_publisher = Marker_Publisher()
    global marker
    marker = Marker()
    
    marker.header.frame_id = "/map"
    marker.header.stamp =marker_publisher.get_clock().now().to_msg()

   
    marker.type = Marker().TEXT_VIEW_FACING
    marker.ns="controller"
    marker.id = 0
        
    marker.action=Marker().ADD
    # Set the scale of the marker
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = 10.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 1.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    controller = marker_publisher.controller
    planner = marker_publisher.planner
    round_num = marker_publisher.round_num
    if controller or planner or round_num:
        marker.text = f"{controller}\n{planner} {round_num}".strip()
    else:
        marker.text = 'None'
    for i in range(4):

        rclpy.spin_once(marker_publisher)
        time.sleep(0.1)

    marker_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
