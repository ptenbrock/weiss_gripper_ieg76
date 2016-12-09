#!/usr/bin/env python
import roslib;
import sys
import rospy
from std_srvs.srv import Trigger, Empty

def send_reference_request():
	rospy.wait_for_service('reference')
	try:
			reference = rospy.ServiceProxy('reference', Trigger)
			resp = reference()
			if resp.success == True:
				print "Success: " + resp.message
			else:
				print "Failure: " + resp.message
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_close_request():
	rospy.wait_for_service('close_jaws')
	try:
			close_jaws = rospy.ServiceProxy('close_jaws', Trigger)
			resp = close_jaws()
			if resp.success == True:
				print "Success: " + resp.message
			else:
				print "Failure: " + resp.message
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_grasp_object_request():
	rospy.wait_for_service('grasp_object')
	try:
			grasp_object = rospy.ServiceProxy('grasp_object', Trigger)
			resp = grasp_object()
			if resp.success == True:
				print "Success: " + resp.message
			else:
				print "Failure: " + resp.message
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_open_request():
	rospy.wait_for_service('open_jaws')
	try:
			open_jaws = rospy.ServiceProxy('open_jaws', Trigger)
			resp = open_jaws()
			if resp.success == True:
				print "Success: " + resp.message
			else:
				print "Failure: " + resp.message
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_ack_error_request():
	rospy.wait_for_service('ack_error')
	try:
			ack_error = rospy.ServiceProxy('ack_error', Trigger)
			resp = ack_error()
			if resp.success == True:
				print "Success: " + resp.message
			else:
				print "Failure: " + resp.message
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_ack_ref_error_request():
	rospy.wait_for_service('ack_ref_error')
	try:
			ack_ref_error = rospy.ServiceProxy('ack_ref_error', Trigger)
			resp = ack_ref_error()
			if resp.success == True:
				print "Success: " + resp.message
			else:
				print "Failure: " + resp.message
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e


def usage():
	print "-------- Commands Weiss Robotics Gripper ieg76 --------"
	print "1. Reference"
	print "2. Open jaws"
	print "3. Close jaws"
	print "4. Grasp object"
	print "5. Ack error"
	print "6. Ack reference error"
	print "7. Exit"

if __name__ == "__main__":
	selected_cmd = usage()
	while True:
		selected_cmd = input("Select a command to send: ")
		if selected_cmd == 1:
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
			print "Sending acknowledge error request..."
			send_ack_error_request()
		elif selected_cmd == 6: 
			print "Sending acknowledge error request..."
			send_ack_ref_error_request()
		elif selected_cmd == 7: 
			sys.exit(0)
		else:
			print "Unknown option entered."