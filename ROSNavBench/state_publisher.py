import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import os



class StatePublisher(Node):

    def __init__(self):
        super().__init__(os.environ["controller"])
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        env_var="controller"    
        if env_var in os.environ:
           controller=os.environ["controller"]
           msg.data =controller +' %d' % self.i
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)
        else:
           msg.data ='NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN' +' %d' % self.i
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    state_publisher = StatePublisher()

    rclpy.spin_once(state_publisher,timeout_sec=2.0)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()