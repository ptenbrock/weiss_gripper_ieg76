#!/usr/bin/env python
import roslib;
import sys
import rospy
from std_srvs.srv import Empty


def send_querry_request():
	 rospy.wait_for_service('querry_device')
	 try:
			 querry_device = rospy.ServiceProxy('querry_device', Empty)
			 resp = querry_device()
	 except rospy.ServiceException, e:
			 print "Service call failed: %s"%e

def send_operate_request():
	 rospy.wait_for_service('operate_device')
	 try:
			 operate_device = rospy.ServiceProxy('operate_device', Empty)
			 resp = operate_device()
	 except rospy.ServiceException, e:
			 print "Service call failed: %s"%e

def send_activate_request():
	 rospy.wait_for_service('activate_device')
	 try:
			 activate_device = rospy.ServiceProxy('activate_device', Empty)
			 resp = activate_device()
	 except rospy.ServiceException, e:
			 print "Service call failed: %s"%e

def send_reference_request():
	 rospy.wait_for_service('reference_device')
	 try:
			 reference_device = rospy.ServiceProxy('reference_device', Empty)
			 resp = reference_device()
	 except rospy.ServiceException, e:
			 print "Service call failed: %s"%e

def send_close_request():
	 rospy.wait_for_service('close_jaws')
	 try:
			 close_jaws = rospy.ServiceProxy('close_jaws', Empty)
			 resp = close_jaws()
	 except rospy.ServiceException, e:
			 print "Service call failed: %s"%e

def send_open_request():
	 rospy.wait_for_service('open_jaws')
	 try:
			 open_jaws = rospy.ServiceProxy('open_jaws', Empty)
			 resp = open_jaws()
	 except rospy.ServiceException, e:
			 print "Service call failed: %s"%e

if __name__ == "__main__":
	 send_querry_request()
	 send_operate_request()
	 send_activate_request()
	 send_reference_request()
	 send_close_request()
	 send_open_request()
	 send_close_request()
