import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('util_arduino_serial')
        self.subscription = self.create_subscription(String, 'arduinoCommand', self.command_callback, 10)

        self.response_publisher = self.create_publisher(String, 'arduinoResponse', 10)

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Adjust the port and baud rate as necessary
            self.get_logger().info("Serial connection established with Arduino.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            exit(1)

        self.create_timer(0.1, self.read_arduino_response)

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        self.serial_port.write(command.encode('utf-8'))
        self.serial_port.write(b'\n')

    def read_arduino_response(self):
        if self.serial_port.in_waiting > 0:
            response = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f"Arduino response: {response}")

            response_msg = String()
            response_msg.data = response
            self.response_publisher.publish(response_msg)

def main(args=None):
    rclpy.init(args=args)
    util_arduino_serial = ArduinoNode()
    rclpy.spin(util_arduino_serial)
    util_arduino_serial.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()