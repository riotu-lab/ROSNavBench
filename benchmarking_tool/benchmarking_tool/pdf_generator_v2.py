import psutil
import rclpy
from rclpy.node import Node
from reportlab.pdfgen import canvas
#from benchmarking_tool.follow_path import main, navigator


class ReportGenerator(Node):
    def __init__(self):
        super().__init__('report_generator')

        # Create a PDF report
        self.report_file = 'report.pdf'
        self.canvas = canvas.Canvas(self.report_file)

        # Get CPU usage
        self.cpu_usage = psutil.cpu_percent()

        # Get memory usage
        self.memory_usage = psutil.virtual_memory().percent

        # Subscribe to Nav2 success status
        #self.subscription = self.create_subscription(
        #    MsgType, 'nav2_success_topic', self.nav2_success_callback, 10    #Change the message type and change the nav2 topic
        #)

        navigator.waitUntilNav2Active()
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        # Call the generate_report() function
        self.generate_report()

    def generate_report(self):
        # Add content to the PDF report
        self.canvas.setFont("Helvetica", 12)
        self.canvas.drawString(50, 800, "ROS 2 Report")
        self.canvas.drawString(50, 750, f"CPU Usage: {self.cpu_usage}%")
        self.canvas.drawString(50, 700, f"Memory Usage: {self.memory_usage}%")
        self.canvas.drawString(50, 650, "Metric 4: Narrow Paths")
        self.canvas.drawString(50, 600, "Metric 5: Execution Time")

        # Save and close the PDF
        self.canvas.save()
        self.get_logger().info('PDF report generated')

        # Destroy the node after generating the report
        self.destroy_node()

    def nav2_success_callback(self, msg):
        if msg.success:
            self.canvas.setFont("Helvetica", 12)
            self.canvas.drawString(50, 550, "Nav2 succeeded in reaching the goal")
            self.canvas.save()


def main(args=None):
    rclpy.init(args=args)

    report_generator = ReportGenerator()

    rclpy.spin(report_generator)
    # Add a condition to stop once the goal is succced
    rclpy.shutdown()


if __name__ == '__main__':
    main()
