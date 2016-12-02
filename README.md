# weiss_gripper_ieg76
This is the ros package of the driver for the Weiss Robotics gripper, model IEG 76-030.

 - Make the python scripts executable
 - Add yourself to the group *dialout* in order to get access the serial port *sudo usermode -a -G dialout $USER*. You need to log out/log in to apply the changes. The serial port is a parameter in the launch file and it is by default */dev/ttyACM0*.
 - Whenever restarting the driver reconnect the USB cable
 - $roslaunch weiss_gripper_ieg76 weiss_gripper_ieg76.launch


