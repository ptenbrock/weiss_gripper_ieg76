#!/usr/bin/env python
import roslib;
import struct
import time
import serial
import rospy
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

ser = serial.Serial('/dev/ttyACM0')  # open serial

def handle_querry_device(req):
	print("Querrying the device.")
	payload = struct.pack(">BBBB", 0x49, 0x44, 0x3f, 0x0a)
	ser.write(payload)
	return EmptyResponse()

def handle_operate_device(req):
	print("Operate the device.")
	payload = struct.pack(">BBBBBBBBBB", 0x4f, 0x50, 0x45, 0x52, 0x41, 0x54, 0x45, 0x28, 0x29, 0x0a)
	ser.write(payload)
	return EmptyResponse()

def handle_activate_device(req):
	print("PDOUT=[03,00] activate the device:")
	payload = struct.pack('>BBBBBBBBBBBBBB', 0x50, 0x44, 0x4f, 0x55, 0x54, 0x3d, 0x5b, 0x30, 0x33, 0x2c, 0x30, 0x30, 0x5d, 0x0a)
	ser.write(payload)
	return EmptyResponse()

def handle_reference_device(req):
	print("PDOUT=[07,00] reference the device:")
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
	serv_querry = rospy.Service('querry_device', Empty, handle_querry_device)
	serv_operate = rospy.Service('operate_device', Empty, handle_operate_device)
	serv_activate = rospy.Service('activate_device', Empty, handle_activate_device)
	serv_ref = rospy.Service('reference_device', Empty, handle_reference_device)
	serv_close = rospy.Service('close_jaws', Empty, handle_close_jaws)
	serv_open = rospy.Service('open_jaws', Empty, handle_open_jaws)
	
	print ser.isOpen()
	print "Ready to receive requests."
	rospy.spin()

if __name__ == "__main__":
	weiss_gripper_ieg76()