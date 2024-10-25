import rclpy
from rclpy.node import Node
from interfaces.srv import EndEffectorCmd
from std_msgs.msg import Int32, String

class EndEffectorNode(Node):
    def __init__(self):
        super().__init__('end_effector_node')

        # Subscribe to the PWM values from Arduino (sent by arduino_serial.py)
        self.pwm_subscription = self.create_subscription(Int32, 'arduinoResponse', self.pwm_callback, 10)

        # Publisher to send motor commands to the Arduino
        self.arduino_command_publisher = self.create_publisher(String, 'arduinoCommand', 10)

        # Service to handle commands from the brain
        self.srv = self.create_service(EndEffectorCmd, 'end_effector_srv', self.handle_end_effector_command)

        # Internal state
        self.simulation_enabled = True  # Toggle to enable/disable torque calculations
        self.screwdriving_in_progress = False
        self.current_pwm = 0
        self.previous_pwm = 0
        self.torque_threshold = 20  # Torque threshold to detect increased resistance
        self.torque_stability_count = 0
        self.torque_stability_limit = 3  # Number of stable readings to confirm end of screwdriving
        self.in_hole = False  # Flag to detect when screw enters the hole
        self.last_command = None

    def handle_end_effector_command(self, request, response):
        command = request.command
    
        # Check for duplicate command
        if self.last_command == command:
            self.get_logger().debug(f"Duplicate command '{command}' not sent.")
            response.success = False
            response.message = f"Duplicate command '{command}' not executed."
            return response

        self.get_logger().info(f"Command: {command}")

        # Handle START SCREWDRIVING command from the Brain
        if command == "START SCREWDRIVING":
            if not self.screwdriving_in_progress:
                self.screwdriving_in_progress = True
                self.in_hole = False  # Reset flag 
                self.send_command_to_arduino("30")  # Start motor

                # Start a timer to simulate motor running for 5 seconds
                self.stop_motor_timer = self.create_timer(5.0, self.stop_motor)
                response.success = True
                response.message = "Screwdriving process started."
            else:
                response.success = False
                response.message = "Screwdriving already in progress."

        # Handle GET_STATUS command
        elif command == "GET_STATUS":
            if self.screwdriving_in_progress:
                response.success = True
                response.message = "Screwdriving in progress."
            elif self.in_hole:
                response.success = True
                response.message = "Screw has entered the hole."
            else:
                response.success = True
                response.message = "Screwdriving completed."

        # Light control commands
        elif command == "TURN_LIGHT_ON":
            self.send_command_to_arduino("LIGHT ON")
            response.success = True
            response.message = "Light turned on."

        elif command == "TURN_LIGHT_OFF":
            self.send_command_to_arduino("LIGHT OFF")
            response.success = True
            response.message = "Light turned off."

        else:
            response.success = False
            response.message = "Unknown command."
            self.get_logger().info("Unknown command.")


        self.last_command = command

        return response

    def pwm_callback(self, msg):
        self.current_pwm = msg.data
        if self.screwdriving_in_progress:
            # Log PWM updates only when there are significant changes or milestones
            self.get_logger().debug(f"Received PWM: {self.current_pwm}")
            self.estimate_torque(self.current_pwm)

    def estimate_torque(self, pwm_value):
        # Calculate torque change
        torque_change = abs(pwm_value - self.previous_pwm)
        self.previous_pwm = pwm_value

        # Evaluate screwdriving behavior
        self.rules(torque_change)

    def rules(self, torque_change):
        """Apply rules for detecting when the screw enters the hole or reaches the end."""
        if not self.in_hole and torque_change > self.torque_threshold:
            self.in_hole = True
            self.get_logger().info("Screw has entered the hole.")

        if torque_change < self.torque_threshold:
            self.torque_stability_count += 1
        else:
            self.torque_stability_count = 0

        if self.torque_stability_count >= self.torque_stability_limit:
            self.screwdriving_in_progress = False
            self.get_logger().info("Screw completed, stopping motor.")
            self.stop_motor()

    def stop_motor(self):
        if self.screwdriving_in_progress:
            self.get_logger().info("Stopping motor.")
            self.send_command_to_arduino("0")
            self.screwdriving_in_progress = False

            # Cancel the timer
            if hasattr(self, 'stop_motor_timer'):
                self.stop_motor_timer.cancel()
        else:
            self.get_logger().debug("Motor already stopped.")

    def send_command_to_arduino(self, command):
        self.get_logger().info(f"Command to Arduino: {command}")
        msg = String()
        msg.data = command
        self.arduino_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()