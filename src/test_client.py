#!/usr/bin/env python
import roslib;
import sys
import rospy
from std_srvs.srv import Trigger, Empty

def send_reference_request():
	rospy.wait_for_service('reference')
	try:
			reference = rospy.ServiceProxy('reference', Empty)
			resp = reference()
			print "Request executed."
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_close_request():
	rospy.wait_for_service('close_jaws')
	try:
			close_jaws = rospy.ServiceProxy('close_jaws', Trigger)
			resp = close_jaws()
			print "Request executed."
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_grasp_object_request():
	rospy.wait_for_service('grasp_object')
	try:
			grasp_object = rospy.ServiceProxy('grasp_object', Trigger)
			resp = grasp_object()
			print "Request executed."
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_open_request():
	rospy.wait_for_service('open_jaws')
	try:
			open_jaws = rospy.ServiceProxy('open_jaws', Trigger)
			resp = open_jaws()
			print "Request executed."
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_close_port_request():
	rospy.wait_for_service('close_port')
	try:
			close_port = rospy.ServiceProxy('close_port', Trigger)
			resp = close_port()
			if resp.success:
				print resp.message
			else:
				print resp.message
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def usage():
	print "-------- Commands Weiss Robotics Gripper ieg76 --------"
	print "1. Reference"
	print "2. Open jaws"
	print "3. Close jaws"
	print "4. Grasp object"
	print "5. Close serial port"
	print "9. Exit"

if __name__ == "__main__":
	selected_cmd = usage()
	while True:
		selected_cmd = input("Select a command to send: ")
		if selected_cmd == 9:
			sys.exit(9)
		elif selected_cmd == 1:
			print "Sending reference request..."
			send_reference_request()
		elif selected_cmd == 2:
			print "Sending open jaws request..."
			send_open_request()
		elif selected_cmd == 3: 
			print "Sending close jaws request..."
			send_close_request()
		elif selected_cmd == 4: 
			print "Sending grasp object request..."
			send_grasp_object_request()
		elif selected_cmd == 5: 
			print "Sending close serial port request..."
			send_close_port_request()
		else:
			print "Unknown option entered."