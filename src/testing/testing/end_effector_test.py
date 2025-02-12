import rclpy
from rclpy.node import Node
from rclpy.task import Future
from interfaces.srv import BrainCmd
from collections import deque 

class BrainEndEffectorTest(Node):
    END_EFFECTOR_MODULE = "endEffector"
    
    def __init__(self):
        super().__init__('end_effector_test')

        # Create a client for the BrainCmd service to interact with the End Effector
        self.brain_cmd_client = self.create_client(BrainCmd, 'brain_srv')

        # Wait until the service is available
        while not self.brain_cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Brain service not available, waiting...')

        # Command queue for multiple commands
        self.command_queue = deque(["TURN_LIGHT_ON", "START_SCREWDRIVING", "TURN_LIGHT_OFF", "GET_STATUS"])
        # self.command_queue = deque(["START_SCREWDRIVING"])
        self.sending_command = False
        self.processing_complete = False 

        # Start processing queue
        self.process_command_queue()

    def process_command_queue(self):
        """Process the next command in the queue."""
        if not self.sending_command and self.command_queue:
            next_command = self.command_queue.popleft()
            self.get_logger().info(f"Processing command: {next_command}")
            self.send_command_request(next_command)
        elif not self.command_queue:
            # Queue is empty, mark processing as complete
            self.get_logger().info("All commands processed, shutting down.")
            self.processing_complete = True  # Set the flag to stop spinning

    def send_command_request(self, cmd):
        try:
            # Create a BrainCmd request
            command_request = BrainCmd.Request()
            command_request.module = self.END_EFFECTOR_MODULE
            command_request.command = cmd

            # Call the service asynchronously and attach the callback
            self.sending_command = True  # Set this to True until the command is processed
            self.get_logger().info(f"Sending command request: {cmd}")
            future = self.brain_cmd_client.call_async(command_request)
            future.add_done_callback(self.command_callback)

        except Exception as e:
            self.get_logger().info(f'ERROR: Could not send Brain service request - {e}')
            self.sending_command = False
            self.process_command_queue()  # Try processing the next command

    def command_callback(self, future: Future):
        try:
            # Retrieve the response from the future object
            response = future.result()
            self.get_logger().info(f'Received brain output data: [{response.output.data}]')

            # Reset sending flag to process the next command
            self.sending_command = False

            # Process the next command after a small delay
            self.create_timer(0.5, self.process_command_queue)

        except Exception as e:
            self.get_logger().info(f'ERROR: Cannot read brain data - {e}')
            self.sending_command = False
            self.process_command_queue()  # Process the next command in case of an error

def main(args=None):
    rclpy.init(args=args)
    node = BrainEndEffectorTest()

    # Run a manual spin loop to check for completion
    while rclpy.ok() and not node.processing_complete:
        rclpy.spin_once(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
