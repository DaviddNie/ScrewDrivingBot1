import rclpy
from rclpy.node import Node
from rclpy.task import Future
from interfaces.srv import VisionCmd, BrainCmd, BrainRoutineCmd
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point, PoseArray
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# The test uses brain to send a command to vision

class BrainRoutineTest(Node):
	BIRDS_EYE_CMD = "birds_eye"
	CALIBRATE_CMD = "calibrate"
	VISION_MODULE = "vision"
	MOVEMENT_MODULE = "movement"
	END_EFFECTOR_MODULE = "endEffector"
	SCREWDRIVING_ROUTINE= "screwdriving"
	
	def __init__(self):
		super().__init__('routine_test')

		# self.vision_status_sub = self.create_subscription(String, 'vision_status', self.vision_status_callback, 10)
		# self.brain_status_sub = self.create_subscription(String, 'brain_status', self.brain_status_callback, 10)

		# self.brain_routine_cmd_client = self.create_client(BrainRoutineCmd, 'brain_routine_srv')

		# Create mutually exclusive callback groups
		self.vision_status_cb_group = MutuallyExclusiveCallbackGroup()
		self.brain_status_cb_group = MutuallyExclusiveCallbackGroup()
		self.brain_routine_cmd_cb_group = MutuallyExclusiveCallbackGroup()

		self.ooi_sub = self.create_subscription(String, 'ooi_server_status', self.ooi_status_callback, 10)
		self.arm_sub = self.create_subscription(String, 'arm_status', self.arm_status_callback, 10)

		# Create subscriptions with callback groups
		self.vision_status_sub = self.create_subscription(
			String,
			'vision_status',
			self.vision_status_callback,
			10,
			callback_group=self.vision_status_cb_group
		)

		self.brain_status_sub = self.create_subscription(
			String,
			'brain_status',
			self.brain_status_callback,
			10,
			callback_group=self.brain_status_cb_group
		)

		# Create client with callback group
		self.brain_routine_cmd_client = self.create_client(
			BrainRoutineCmd,
			'brain_routine_srv',
			callback_group=self.brain_routine_cmd_cb_group
		)

		while not self.brain_routine_cmd_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Brain routine client service not available, waiting...')

		self.send_command_request(self.SCREWDRIVING_ROUTINE)

	def send_command_request(self, cmd):
		try:
			command_request = BrainRoutineCmd.Request()
			command_request.command = cmd

			future = self.brain_routine_cmd_client.call_async(command_request)
			future.add_done_callback(self.command_callback)
		except Exception as e:
			self.get_logger().info(f'ERROR: Could not send Brain service request - {e}')

	def vision_status_callback(self, msg):
		try:
			self.get_logger().info(f'Vision Status - {msg.data}')
		except Exception as e:
			self.get_logger().error(f'ERROR: Cannot read vision_status topic data - {e}')

	def ooi_status_callback(self, msg):
		try:
			self.get_logger().info(f'OOI Status - {msg.data}')
		except Exception as e:
			self.get_logger().error(f'ERROR: Cannot read ooi_status topic data - {e}')


	def brain_status_callback(self, msg):
		try:
			self.get_logger().info(f'Brain Status - {msg.data}')
		except Exception as e:
			self.get_logger().error(f'ERROR: Cannot read brain_status topic data - {e}')

	def arm_status_callback(self, msg):
		try:
			self.get_logger().info(f'Arm Status - {msg.data}')
		except Exception as e:
			self.get_logger().error(f'ERROR: Cannot read brain_status topic data - {e}')


	def command_callback(self, future: Future):
		try:
			response = future.result()
			
			self.get_logger().info(f'Received brain output data: [{response.output.data}]')

		except Exception as e:
			self.get_logger().info(f'ERROR: Cannot get brain output data - {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BrainRoutineTest()

    # Use MultiThreadedExecutor with multiple threads
    executor = MultiThreadedExecutor(num_threads=4)  # Ensures parallel processing
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
	main()
