#!/usr/bin/env python
import roslib;
import sys
import rospy
from std_srvs.srv import Empty


def send_query_request():
	rospy.wait_for_service('query')
	try:
			query = rospy.ServiceProxy('query', Empty)
			resp = query()
			print "Request executed."
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_operate_request():
	rospy.wait_for_service('operate')
	try:
			operate = rospy.ServiceProxy('operate', Empty)
			resp = operate()
			print "Request executed."
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_activate_request():
	rospy.wait_for_service('activate')
	try:
			activate = rospy.ServiceProxy('activate', Empty)
			resp = activate()
			print "Request executed."
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

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
			close_jaws = rospy.ServiceProxy('close_jaws', Empty)
			resp = close_jaws()
			print "Request executed."
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def send_open_request():
	rospy.wait_for_service('open_jaws')
	try:
			open_jaws = rospy.ServiceProxy('open_jaws', Empty)
			resp = open_jaws()
			print "Request executed."
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

def usage():
	print "-------- Commands Weiss Robotics Gripper ieg76 --------"
	print "1. Query"
	print "2. Operate"
	print "3. Activate"
	print "4. Reference"
	print "5. Open jaws"
	print "6. Close jaws"
	print "0. Exit"

if __name__ == "__main__":
	selected_cmd = usage()
	while True:
		selected_cmd = input("Select a command to send: ")
		if selected_cmd == 0:
			sys.exit(0)
		elif selected_cmd == 1:
			print "Sending query request..."
			send_query_request()
		elif selected_cmd == 2:
			print "Sending operate request..."
			send_operate_request()
		elif selected_cmd == 3:
			print "Sending activate request..."
			send_activate_request()
		elif selected_cmd == 4:
			print "Sending reference request..."
			send_reference_request()
		elif selected_cmd == 5:
			print "Sending open jaws request..."
			send_open_request()
		elif selected_cmd == 6: 
			print "Sending close jaws request..."
			send_close_request()
		else:
			print "Unknown option entered."