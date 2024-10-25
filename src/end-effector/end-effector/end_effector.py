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
        self.screwdriving_in_progress = False
        self.current_pwm = 0
        self.previous_pwm = 0
        self.torque_threshold = 20  # Torque threshold to detect increased resistance
        self.torque_stability_count = 0
        self.torque_stability_limit = 3  # Number of stable readings to confirm end of screwdriving
        self.in_hole = False  # Flag to detect when screw enters the hole

    def handle_end_effector_command(self, request, response):
        command = request.command
        self.get_logger().info(f"Received End Effector Command: {command}")

        # Handle START SCREWDRIVING command from the Brain
        if command == "START SCREWDRIVING":
            if not self.screwdriving_in_progress:
                self.screwdriving_in_progress = True
                self.in_hole = False  # Reset flag for new screwdriving session
                self.send_command_to_arduino("START")  # Command to start motor via Arduino
                response.success = True
                response.message = "Screwdriving process started."
            else:
                response.success = False
                response.message = "Screwdriving already in progress."
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
        else:
            response.success = False
            response.message = "Unknown command."

        return response

    def pwm_callback(self, msg):
        # Handle PWM updates from the Arduino
        self.current_pwm = msg.data
        self.get_logger().info(f"Received PWM: {self.current_pwm}")

        if self.screwdriving_in_progress:
            self.estimate_torque(self.current_pwm)

    def estimate_torque(self, pwm_value):
        # Calculate torque change based on the difference between current and previous PWM values
        torque_change = abs(pwm_value - self.previous_pwm)
        self.previous_pwm = pwm_value

        # Call rules function to evaluate screwdriving behavior
        self.rules(torque_change)

    def rules(self, torque_change):
        """Applies the rules for detecting when the screw enters the hole or reaches the end."""
        # Check if torque indicates that screw is in the hole
        if not self.in_hole and torque_change > self.torque_threshold:
            self.in_hole = True
            self.get_logger().info("Screw has entered the hole.")
        
        # Detect the end of the hole based on stability of high torque readings
        if torque_change < self.torque_threshold:
            self.torque_stability_count += 1
        else:
            self.torque_stability_count = 0  # Reset if torque changes significantly

        # Stop motor if torque has remained stable, indicating screw completion
        if self.torque_stability_count >= self.torque_stability_limit:
            self.screwdriving_in_progress = False
            self.get_logger().info("Screw Completed")
            self.stop_motor()

    def stop_motor(self):
        # Publish a command to the Arduino to stop the motor
        self.get_logger().info("Stopping motor (end of screwdriving).")
        self.send_command_to_arduino("0")  # Send '0' to stop the motor

    def send_command_to_arduino(self, command):
        # Send a command to the Arduino to control the motor or light
        self.get_logger().info(f"Sending command to Arduino: {command}")
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
