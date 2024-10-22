import rclpy
from rclpy.node import Node
from rclpy.task import Future
from interfaces.srv import VisionCmd
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point, PoseArray
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
import time

class VisionTest(Node):
	BIRDS_EYE_CMD = "birds_eye"
	CALIBRATE_CMD = "calibrate"
	def __init__(self):
		super().__init__('vision_test')

		self.vision_status_sub = self.create_subscription(String, 'vision_status', self.vision_status_callback, 10)

		self.vision_cmd_client = self.create_client(VisionCmd, 'vision_srv')

		while not self.vision_cmd_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Command service not available, waiting...')

		self.send_command_request(self.BIRDS_EYE_CMD)

	def send_command_request(self, cmd):
		try:
			command_request = VisionCmd.Request()
			command_request.command = cmd

			future = self.vision_cmd_client.call_async(command_request)
			future.add_done_callback(self.command_callback)
		except Exception as e:
			self.get_logger().info(f'ERROR: Could not send Command service request - {e}')

	def vision_status_callback(self, msg):
		self.get_logger().info(f'Vision Status - {msg.data}')

	def command_callback(self, future: Future):
		try:
			# Convert the PoseArray data (list of integers) to a string
			data_str = ', '.join(map(str, future.poses))
			self.get_logger().info(f'Received vision data: [{data_str}]')
		except Exception as e:
			self.get_logger().info(f'ERROR: Cannot read vision data - {e}')

def main(args=None):
	rclpy.init(args=args)
	node = VisionTest()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
