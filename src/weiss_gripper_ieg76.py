#!/usr/bin/env python
import roslib;
import struct
import time
import serial
import rospy
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

ser = serial.Serial('/dev/ttyACM0')  #open serial

def handle_query(req):
	print("Query:")
	payload = struct.pack(">BBBB", 0x49, 0x44, 0x3f, 0x0a)
	ser.write(payload)
	return EmptyResponse()

def handle_operate(req):
	print("Operate:")
	payload = struct.pack(">BBBBBBBBBB", 0x4f, 0x50, 0x45, 0x52, 0x41, 0x54, 0x45, 0x28, 0x29, 0x0a)
	ser.write(payload)
	return EmptyResponse()

def handle_activate(req):
	print("PDOUT=[03,00] activate:")
	payload = struct.pack('>BBBBBBBBBBBBBB', 0x50, 0x44, 0x4f, 0x55, 0x54, 0x3d, 0x5b, 0x30, 0x33, 0x2c, 0x30, 0x30, 0x5d, 0x0a)
	ser.write(payload)
	return EmptyResponse()

def handle_reference(req):
	print("PDOUT=[07,00] reference:")
	payload = struct.pack('>BBBBBBBBBBBBBB', 0x50, 0x44, 0x4f, 0x55, 0x54, 0x3d, 0x5b, 0x30, 0x37, 0x2c, 0x30, 0x30, 0x5d, 0x0a)
	ser.write(payload)
	return EmptyResponse()

def handle_close_jaws(req):
	print("PDOUT=[03,00] close the jaws:")
	payload = struct.pack('>BBBBBBBBBBBBBB', 0x50, 0x44, 0x4f, 0x55, 0x54, 0x3d, 0x5b, 0x30, 0x33, 0x2c, 0x30, 0x30, 0x5d, 0x0a)
	ser.write(payload)
	return EmptyResponse()

def handle_open_jaws(req):
	print("PDOUT=[02,00] open the jaws:")
	payload = struct.pack('>BBBBBBBBBBBBBB', 0x50, 0x44, 0x4f, 0x55, 0x54, 0x3d, 0x5b, 0x30, 0x32, 0x2c, 0x30, 0x30, 0x5d, 0x0a)
	ser.write(payload)
	return EmptyResponse()	

def weiss_gripper_ieg76():
	rospy.init_node('weiss_gripper_ieg76_node')
	serv_query = rospy.Service('query', Empty, handle_query)
	serv_operate = rospy.Service('operate', Empty, handle_operate)
	serv_activate = rospy.Service('activate', Empty, handle_activate)
	serv_ref = rospy.Service('reference', Empty, handle_reference)
	serv_close = rospy.Service('close_jaws', Empty, handle_close_jaws)
	serv_open = rospy.Service('open_jaws', Empty, handle_open_jaws)
	
	print "Serial port opened: " + str(ser.isOpen())
	print "Ready to receive requests."
	rospy.spin()

if __name__ == "__main__":
	weiss_gripper_ieg76()