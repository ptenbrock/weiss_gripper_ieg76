# weiss_gripper_ieg76
This is the ROS package of the driver for the [Weiss Robotics gripper, model IEG 76-030](https://www.weiss-robotics.com/en/produkte/gripping-systems/integration-line-en/ieg-en/).

 1. [Device Configurator Windows](#device-configurator-windows)
	 1. [Installation](#installation)
	 2. [Configuration](#configuration)
	 3. [Usage](#usage)
 2. [Linux Driver](#linux-driver)
	 1. [Driver Installation](#driver-installation)
	 2. [Start-up](#start-up)
	 3. [Driver Usage](#driver-usage)
		 1. [Reference](#reference)
		 2. [Open jaws](#open-jaws)
		 3. [Close jaws](#close-jaws)
		 4. [Ack error](#ack-error)
		 5. [Ack reference error](#ack-reference-error)
		 6. [Exit](#exit)
		 7. [Reconnect](#reconnect)
		 8. [Shutdown](#shutdown)


##Device Configurator in Windows
The gripper comes with a CD containing the application *Device Configurator* that can be installed and used in Windows to configure and test the gripper.

###Installation
Follow the installation procedures described in the *Installationsguide.pdf* on the CD. If you get the error "The INF file you selected does not support this method of installation"  while installing *dciolink.inf* then:

 1. Run the installer of the *Device Configurator* from the CD
 2. Go to the *Device Manager* in *Control Panel* and disable/enable the COM port allocated for the device under *Ports(COM&LPT)*
 3. Launch the application *Device Configurator*

###Configuration
The gripper provides 4 gripping configurations. The linux driver uses the first configuration *Griff 0*. To configure it click on the button "Parameter" and edit the values for *Griff 0*.

1. Launch the application *Device Configurator*
2. Select the port allocated for the device
3. Click the button *Verbinden*
4. Click on the tab *Steuerung* and then click on the parameter that you would like to modify *Öffnungsposition*, *Schließposition* or *Greifkraft*. Another possibility is to click on the tab *Parameter* and edit the values.
5. Click the button *Trennen*
6. Close the application "Device Configurator"

###Usage

 1. Launch the application *Device Configurator*
 2. Select the port allocated for the device
 3. Click the button *Verbinden*
 4. Click the button *Aktivieren*
 5. Click the button *Referenzieren*
 6. Click the buttons *Schließen* / *Öffnen* to close/open the gripper
 7. Click the button *Deaktivieren*
 8. Click the button *Trennen*

##Linux Driver
###Driver Installation

 1. Navigate on your local machine to the *src* folder of your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and clone this repository:
 
		$ cd ~/catkin_ws/src
		$ git clone https://github.com/ipa-lth/weiss_gripper_ieg76.git
 
 2. Navigate to the root of your catkin workspace and "build" your workspace:
	
		$ cd ..
		$ catkin_make
 
 3. Source your setup file to overlay the workspace on top of your environment:

		$ source devel/setup.bash

	To check that the ROS package has been successfully sourced execute the command `rospack find weiss_gripper_ieg76` and the absolute path to the ROS package *weiss_gripper_ieg76* should be displayed in the terminal.

 4. Add yourself to the group *dialout* in order to get access the serial port `sudo adduser $USER dialout`.

 You need to log-out/log-in to apply the changes.
 
###Start-up

 1. The driver uses by default the port */dev/ttyACM0* to communicate with the gripper. Issue the command

 `ls /dev/tty*`

 before and after pluging-in the USB cabel into your computer to determine the port allocated for the device. Besides */dev/ttyACM0*, another common allocated port is */dev/ttyACM1*. If this is the case then open the file *weiss_gripper_ieg76.launch* and change the line:

		<param name="serial_port_address" value="/dev/ttyACM0"/>
to

		<param name="serial_port_address" value="/dev/ttyACM1"/>

 2. Execute `$roscore` to start *roscore* in a separate terminal. By starting *roscore* separately you do not have to kill and restart *rostopic* whenever the driver restarts or reconnects in order to display the messages published on the topics.
 
 3. Start the driver by executing the command:

		$ roslaunch weiss_gripper_ieg76 weiss_gripper_ieg76.launch

 4. The ROS package contains an application for testing the interaction with the gripper. Start it as a ROS node by executing the command:

		$ rosrun weiss_gripper_ieg76 test_client.py

 5. Display the messages published on the */joint_states* topic:
 
		$ rostopic echo /joint_states
 
 6. Display the messages published on the */diagnostics* topic:

		$ rostopic echo /diagnostics

###Driver Usage

The *test_client* provides a simple menu for interacting with the gripper:

> -------- Commands Weiss Robotics Gripper ieg76 --------
> 1. Reference
> 2. Open jaws
> 3. Close jaws
> 4. Grasp object
> 5. Ack error
> 6. Ack reference error
> 7. Exit 
> Select a command to send:

####Reference
This is the first command that should be sent to the gripper after it is powered up. It performs a reference motion by completely opening the gripper's jaws. In case an error occurs during the reference it can be acknowledge by selecting option *6. Ack reference error* in the menu and the reference procedure must be performed again. After the gripper has been successfully referenced the closing/opening of the jaws and object grasping can be triggered.

####Open jaws
This option in the menu triggers the opening of the gripper's jaws. In case an error occurs during the opening of the jaws the gripper's light will switch its color from blue to red. Use the menu option *5. Ack error* to acknowledge the error (the gripper's light will switch its color from red to blue) and try again.

####Close jaws
This option in the menu triggers the closing of the gripper's jaws. In case an error occurs during the opening of the jaws the gripper's light will switch its color from blue to red. Use the menu option *5. Ack error* to acknowledge the error (the gripper's light will switch its color from red to blue) and try again.

####Grasp object
Use this option to grasp an object. When an object is grasped the gripper's light will switch it's color from blue to green. If there is no object between the jaws for the gripper to grasp the light will remain blue and a message will be printed to inform the user. In case an error occurs use the menu option *5. Ack error* to acknowledge it.

####Ack error
Use this option to acknowledge any error that may occur during opening/closing of the gripper or while grasping an object. When an error occurs the gripper's light will turn red. After acknowledging the error the light will switch its color to blue.

**Note**: For testing purposes an error can be **sometimes** be triggered by following this procedure:

 - Unplug the power source and USB cable and then plug them back in
 - Launch the driver and start the test_client
 - Perform a reference motion. After the reference the jaws are completely opened.
 - Now issue an open jaws command and sometimes an error occurs.
 - Use the menu option *5. Ack error* since the error occurred during opening of the gripper's jaws 

####Ack reference error
Use this option to acknowledge any error that may occur during the reference procedure. When an error occurs the gripper's light will turn red. After acknowledging the error the light will switch its color to blue.

####Exit
This option in the menu closes the *test_client* application. It does not shutdown the driver. To shutdown the driver switch to the terminal where the driver is running and press *Ctrl + C.*

####Reconnect
The driver will automatically try to reconnect to the serial port if it detects a communication problem. For example, while the driver is up and running unplug the USB cable and then plug it back again.

###Shutdown
To shutdown the driver switch to the terminal where the driver is running and press *Ctrl + C*.  The driver will gracefully shutdown. 
