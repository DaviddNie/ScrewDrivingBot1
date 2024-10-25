# util_arduino_serial

This is a utility function which allows easy communication between ros and an arduino.
It opens a serial communcation port and passess through string message published to 'arduinoCommand'.

To run the node use the command:

    ros2 run util_arduino_serial util_arduino_serial 

*Note: requires an arduino to be connected to ubuntu to run*

require: 'pip install pyserial'

To find arduino port, Go to Arduino and then Tools > Port.


To talk to the arduino, publish to /arduinoCommand

Example: ros2 topic pub /arduinoCommand std_msgs/String "data: 'LIGHT ON'"
