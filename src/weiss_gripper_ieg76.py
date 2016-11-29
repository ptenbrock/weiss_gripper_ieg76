#!/usr/bin/env python
import roslib;
import struct
import time
import serial
import rospy
import threading
from serial import SerialException
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

ser = serial.Serial()
serial_port_lock = threading.Lock()

def handle_query(req):
	print("Query:")
	payload = struct.pack(">BBBB", 0x49, 0x44, 0x3f, 0x0a)
	ser.write(payload)
	time.sleep(0.5)
	return EmptyResponse()

def handle_operate(req):
	print("Operate:")
	payload = struct.pack(">BBBBBBBBBB", 0x4f, 0x50, 0x45, 0x52, 0x41, 0x54, 0x45, 0x28, 0x29, 0x0a)
	ser.write(payload)
	time.sleep(0.5)
	return EmptyResponse()

def handle_activate(req):
	print("PDOUT=[03,00] activate:")
	payload = struct.pack('>BBBBBBBBBBBBBB', 0x50, 0x44, 0x4f, 0x55, 0x54, 0x3d, 0x5b, 0x30, 0x33, 0x2c, 0x30, 0x30, 0x5d, 0x0a)
	ser.write(payload)
	time.sleep(0.5)
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

def handle_close_port(req):
	print("Close the serial port:")
	serial_port_lock.acquire()
	try: 
		if ser.isOpen():
			ser.close()
	except SerialException as e:
		print "error closing serial port: " + str(e)
	finally:
		serial_port_lock.release()
	return EmptyResponse()

def read_from_port(ser):
	while True:
		serial_port_lock.acquire()
		if ser.isOpen():
			try:
				if (ser.inWaiting()>0): #if incoming bytes are waiting to be read from the serial input buffer
					data_str = ser.read(ser.inWaiting()).decode('ascii') #read the bytes and convert from binary array to ASCII
					print(data_str)
			except Exception as e:
				print "error reading from the serial port: " + str(e)
			finally:
				serial_port_lock.release()

def weiss_gripper_ieg76():
	rospy.init_node('weiss_gripper_ieg76_node')
	serial_port_addr = rospy.get_param("~serial_port_address", '/dev/ttyACM0')
	ser.port = serial_port_addr
	ser.timeout = 0
	is_serial_port_opened = False
	while not is_serial_port_opened:
		try: 
			ser.open()
			is_serial_port_opened = True
		except Exception as e:
			is_serial_port_opened = False
			print "\terror opening serial port " + serial_port_addr + ": " + str(e)
			print "Retrying to open the serial port " + serial_port_addr + "..."
			time.sleep(1)
	
	serv_query = rospy.Service('query', Empty, handle_query)
	serv_operate = rospy.Service('operate', Empty, handle_operate)
	serv_activate = rospy.Service('activate', Empty, handle_activate)
	serv_ref = rospy.Service('reference', Empty, handle_reference)
	serv_close = rospy.Service('close_jaws', Empty, handle_close_jaws)
	serv_open = rospy.Service('open_jaws', Empty, handle_open_jaws)
	serv_close_port = rospy.Service('close_port', Empty, handle_close_port)
	
	print "Serial port opened: " + str(ser.isOpen())
	print "Ready to receive requests."

	read_thread = threading.Thread(target=read_from_port, args=(ser,))
	read_thread.daemon = True
	read_thread.start()
	rospy.spin()

if __name__ == "__main__":
	weiss_gripper_ieg76()