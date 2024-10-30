import rclpy
from rclpy.node import Node
from rclpy.task import Future
from interfaces.srv import VisionCmd, BrainCmd, RealCoorCmd
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point, PoseArray
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
import time

# The test uses brain to send a command to vision

class BrainVisionTest(Node):
	BIRDS_EYE_CMD = "birds_eye"
	CALIBRATE_CMD = "calibrate"
	VISION_MODULE = "vision"
	MOVEMENT_MODULE = "movement"
	END_EFFECTOR_MODULE = "endEffector"
	OOI_MODULE = "ooi"
	
	def __init__(self):
		super().__init__('vision_test')

		self.vision_status_sub = self.create_subscription(String, 'vision_status', self.vision_status_callback, 10)
		self.brain_status_sub = self.create_subscription(String, 'brain_status', self.brain_status_callback, 10)
		self.ooi_sub = self.create_subscription(String, 'ooi_server_status', self.ooi_status_callback, 10)

		self.brain_cmd_client = self.create_client(BrainCmd, 'brain_srv')

		while not self.brain_cmd_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Brain service not available, waiting...')

		# self.send_command_request(self.BIRDS_EYE_CMD)

# [INFO] [1730288014.038506720] [vision_test]: Vision Status - Point 1 is at pixel x = 183.86, y = 149.86
# [INFO] [1730288014.041444540] [vision_test]: Vision Status - Point 2 is at pixel x = 132.65, y = 128.03
# [INFO] [1730288014.043445586] [vision_test]: Vision Status - Point 3 is at pixel x = 177.87, y = 96.32

		self.pose = Pose()
		self.pose.position.x = 183.86
		self.pose.position.y = 149.86
		self.pose.position.z = 7

		self.send_OOI_request(self.pose)

	def send_command_request(self, cmd):
		try:
			command_request = BrainCmd.Request()
			command_request.module = self.VISION_MODULE

			future = self.brain_cmd_client.call_async(command_request)
			future.add_done_callback(self.command_callback)
		except Exception as e:
			self.get_logger().info(f'ERROR: Could not send Brain service request - {e}')

	def send_OOI_request(self, pose):
		try:
			command_request = BrainCmd.Request()
			command_request.module = self.VISION_MODULE
			command_request.img = pose

			future = self.brain_cmd_client.call_async(command_request)
			future.add_done_callback(self.command_callback)
		except Exception as e:
			self.get_logger().info(f'ERROR: Could not send Brain service request - {e}')

	def vision_status_callback(self, msg):
		try:
			self.get_logger().info(f'Vision Status - {msg.data}')
		except Exception as e:
			self.get_logger().error(f'ERROR: Cannot read vision_status topic data - {e}')

	def brain_status_callback(self, msg):
		try:
			self.get_logger().info(f'Brain Status - {msg.data}')
		except Exception as e:
			self.get_logger().error(f'ERROR: Cannot read brain_status topic data - {e}')

	def ooi_status_callback(self, msg):
		try:
			self.get_logger().info(f'OOI Status - {msg.data}')
		except Exception as e:
			self.get_logger().error(f'ERROR: Cannot read ooi_status topic data - {e}')

	def command_callback(self, future: Future):
		try:
			response = future.result()
			
			self.get_logger().info(f'Received brain output data: [{response.output.data}]')

		except Exception as e:
			self.get_logger().info(f'ERROR: Cannot read vision data - {e}')

def main(args=None):
	rclpy.init(args=args)
	node = BrainVisionTest()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
