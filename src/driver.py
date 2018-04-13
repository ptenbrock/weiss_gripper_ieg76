<#!/usr/bin/env python
# import roslib
# import struct
# import time
import serial
# import rospy
import threading
# import diagnostic_updater
# from serial import SerialException
# from std_srvs.srv import Trigger, TriggerResponse
# from weiss_gripper_ieg76.srv import *
# from sensor_msgs.msg import JointState
# from diagnostic_msgs.msg import DiagnosticStatus


class Driver(object):
	def __init__(self):
		rospy.on_shutdown(self.shutdown_handler)
		serial_port_addr = rospy.get_param("~serial_port_address", '/dev/ttyACM0')
		self.serial_port_comm = SerialPortComm(serial_port_addr, serial_timeout=0)

		self.driver_logic = DriverLogic(self.serial_port_comm)

		serv_ref = rospy.Service('move', Trigger, self.handle_reference)
		serv_ref = rospy.Service('open', Trigger, self.handle_reference)
		serv_ref = rospy.Service('close', Trigger, self.handle_reference)
		serv_ref = rospy.Service('grasp', Trigger, self.handle_reference)
		# serv_ref = rospy.Service('release', Trigger, self.handle_reference)
		serv_ref = rospy.Service('reference', Trigger, self.handle_reference)
		serv_ref = rospy.Service('ack', Trigger, self.handle_reference)
		serv_ref = rospy.Service('set_force', Trigger, self.handle_reference)

		self.states_publisher_thread = StatesPublisher(0.8, self.serial_port_comm)

	def handle_reference(self, req):
		reply = TriggerResponse()
		self.driver_logic.service_called(transition="do_reference", params=req, trigger_response=reply)
		return reply

	def shutdown_handler(self):
		self.states_publisher_thread.shutdown()
		self.driver_logic.shutdown()
		self.serial_port_comm.shutdown()
		rospy.loginfo("Gracefully shutting down the driver...")

	def run(self):
		self.serial_port_comm.daemon = True
		self.states_publisher_thread.daemon = True

		rospy.logdebug("Starting threads...")
		self.serial_port_comm.start()
		self.states_publisher_thread.start()
		rospy.logdebug("Threads started.")

		rospy.loginfo("Ready to receive requests.")

		rospy.spin()


if __name__ == "__main__":
	#rospy.init_node('driver_node', log_level=rospy.DEBUG)
	rospy.init_node('ieg_driver_node')

	driver = Driver()
	driver.run()
