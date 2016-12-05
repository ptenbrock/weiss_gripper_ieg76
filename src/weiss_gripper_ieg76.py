#!/usr/bin/env python
import roslib
import struct
import time
import serial
import rospy
import threading
import diagnostic_updater
from serial import SerialException
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus 

ser = serial.Serial()
serial_port_lock = threading.Lock()

status_flags_lock = threading.Lock()
fresh_flags_cond_var = threading.Condition()#this uses the lock created by default
reference_cond_var = threading.Condition()
jaws_closed_cond_var = threading.Condition()
jaws_opened_cond_var = threading.Condition()
object_grasped_cond_var = threading.Condition()
fault_cond_var = threading.Condition()

POS = 0.0
OPEN_FLAG = 0b0
OLD_OPEN_FLAG = 0b0
CLOSED_FLAG = 0b0
OLD_CLOSED_FLAG = 0b0
HOLDING_FLAG = 0b0
OLD_HOLDING_FLAG = 0b0
FAULT_FLAG = 0b0
IDLE_FLAG = 0b0
OLD_IDLE_FLAG = 0b0
TEMPFAULT_FLAG = 0b0
TEMPWARN_FLAG = 0b0
MAINT_FLAG = 0b0

shutdown_driver = False

def log_debug_flags():
	rospy.logdebug("POS = %f", POS)
	rospy.logdebug("IDLE_FLAG = %s", IDLE_FLAG)
	rospy.logdebug("OPEN_FLAG = %s", OPEN_FLAG)
	rospy.logdebug("CLOSED_FLAG = %s", CLOSED_FLAG)
	rospy.logdebug("HOLDING_FLAG = %s", HOLDING_FLAG)
	rospy.logdebug("FAULT_FLAG = %s", FAULT_FLAG)
	rospy.logdebug("TEMPFAULT_FLAG = %s", TEMPFAULT_FLAG)
	rospy.logdebug("TEMPWARN_FLAG = %s", TEMPWARN_FLAG)
	rospy.logdebug("MAINT_FLAG = %s", MAINT_FLAG)

def create_send_payload(command):
	cmd_dict = {"query":"ID?\n", "activate":"PDOUT=[02,00]\n", "operate":"OPERATE()\n", "close":"PDOUT=[03,00]\n", "open":"PDOUT=[02,00]\n", "reference":"PDOUT=[07,00]\n", "deactivate":"PDOUT=[00,00]\n", "fallback":"PDOUT=FALLBACK(1)\n", "mode":"MODE?\n", "restart":"RESTART()\n", "reset":"PDOUT=[00,00]\n"}
	# Query if command is known
	if not(command in cmd_dict):
		rospy.logerr("Command not recognized")
		return None

	send_cmd = cmd_dict[command]

	# create format for command
	fmt = '>'
	for i in range(len(send_cmd)):
		fmt+='B'

	# create bytelist from command
	bytelist =[ord(c) for c in send_cmd]
	# create payload
	payload = struct.pack(fmt,*bytelist)
	return payload


class serial_port_reader(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)

	def extract_info(self, read_data_hexstr):
		fresh_flags_cond_var.acquire()
		#rospy.logdebug("serial_port_reader inside extract_info")
		global POS
		global OPEN_FLAG 
		global OLD_OPEN_FLAG
		global CLOSED_FLAG 
		global OLD_CLOSED_FLAG 
		global HOLDING_FLAG
		global OLD_HOLDING_FLAG 
		global FAULT_FLAG
		global IDLE_FLAG
		global OLD_IDLE_FLAG
		global TEMPFAULT_FLAG
		global TEMPWARN_FLAG
		global MAINT_FLAG
		#the data read from the serial port is @PDIN=[BYTE0,BYTE1,BYTE2,BYTE3] (see pag.20 in user manual)
		position_hexstr = read_data_hexstr[7:9] + read_data_hexstr[10:12] #remove the comma "," bewteen "BYTE0" and "BYTE1"
		POS = int(position_hexstr, 16) / float(100) #position in mm

		byte3_hexstr = read_data_hexstr[16:18]
		byte3_binary = int(byte3_hexstr, 16)
		mask = 0b1
		#rospy.logdebug("serial_port_reader reference_cond_var.acquire()")
		reference_cond_var.acquire()
		try:
			#rospy.logdebug("serial_port_reader has acquired reference_cond_var")
			OLD_IDLE_FLAG = IDLE_FLAG
			IDLE_FLAG = byte3_binary & mask
			if OLD_IDLE_FLAG == 0 and IDLE_FLAG == 1:
				# signal the event
				reference_cond_var.notify()
				rospy.logdebug("Reference event triggered")
		finally:
			#rospy.logdebug("serial_port_reader releasing reference_cond_var")	
			reference_cond_var.release()	

		byte3_binary = byte3_binary >> 1
		jaws_opened_cond_var.acquire()
		try:
			OLD_OPEN_FLAG = OPEN_FLAG
			OPEN_FLAG = byte3_binary & mask
			if OLD_OPEN_FLAG == 0 and OPEN_FLAG == 1:
				#the transition from jaws not opened to jaws opened has occured. Signal this event.
				jaws_opened_cond_var.notify()
				rospy.logdebug("Open event triggered")
		finally:
			jaws_opened_cond_var.release()

		byte3_binary = byte3_binary >> 1
		jaws_closed_cond_var.acquire()
		try:
			OLD_CLOSED_FLAG = CLOSED_FLAG
			CLOSED_FLAG = byte3_binary & mask
			if OLD_CLOSED_FLAG == 0 and CLOSED_FLAG == 1:
				#the transition from jaws not closed to jaws closed has occured. Signal this event.
				jaws_closed_cond_var.notify()
				rospy.logdebug("Close event triggered")
		finally:
			jaws_closed_cond_var.release()

		byte3_binary = byte3_binary >> 1
		object_grasped_cond_var.acquire()
		try:
			OLD_HOLDING_FLAG = HOLDING_FLAG
			HOLDING_FLAG = byte3_binary & mask
			if OLD_HOLDING_FLAG == 0 and HOLDING_FLAG == 1:
				#the transition from not holding/grasping an object to holding/grasping an object has occured. Signal this event.
				object_grasped_cond_var.notify()
				rospy.logdebug("Grasp event triggered")
		finally:
			object_grasped_cond_var.release()

		byte3_binary = byte3_binary >> 1
		fault_cond_var.acquire()
		try:
			FAULT_FLAG = byte3_binary & mask
		finally:
			fault_cond_var.release()

		byte3_binary = byte3_binary >> 1
		TEMPFAULT_FLAG = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		TEMPWARN_FLAG = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		MAINT_FLAG = byte3_binary & mask

		fresh_flags_cond_var.notify()# signal to states_publisher that new data is available for publishing
		fresh_flags_cond_var.release()

	def reconnect_serial_port(self):
		global ser
		serial_port_addr = ser.port
		if ser.isOpen(): 
			ser.close()
		ser = None
		time.sleep(2)
		ser = serial.Serial()
		ser.port = serial_port_addr
		ser.timeout = 0
		is_serial_port_opened = False

		while not is_serial_port_opened:
			try: 
				ser.open()
				ser.flushInput()
				ser.flushOutput()
				is_serial_port_opened = True
			except Exception as e:
				is_serial_port_opened = False
				rospy.logerr("Error opening serial port %s: %s", serial_port_addr, e)
				rospy.loginfo("Retrying to open the serial port %s...", serial_port_addr)
				time.sleep(1)
		rospy.loginfo("Serial port opened: %s", is_serial_port_opened)

		rospy.loginfo("Query")
		payload = create_send_payload("query")
		try:
			ser.write(payload)
			time.sleep(0.5)
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Fallback")
			payload = create_send_payload("fallback")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Mode")
			payload = create_send_payload("mode")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Restart")
			payload = create_send_payload("restart")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Operate")
			payload = create_send_payload("operate")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Reset flags")
			payload = create_send_payload("reset")
			ser.write(payload)
			time.sleep(0.5)
		except Exception as e:
			rospy.logerr("Error reading from the serial port while reconnect: %s", e)

	def run(self):
		#read from port
		connection_errors_no = 0
		incoming_bytes_no = 0
		serial_port_status = ser.isOpen()
		rospy.logdebug("serial_port_reader.run() serial_port_status = %s", serial_port_status)
		while (not shutdown_driver) and ser.isOpen():
			try:
				incoming_bytes_no = ser.inWaiting()
				
				if (incoming_bytes_no>0): #if incoming bytes are waiting to be read from the serial input buffer
					input_data = ser.read(ser.inWaiting())
					data_str = input_data.decode('ascii') #read the bytes and convert from binary array to ASCII
					
					if incoming_bytes_no == 22:
						self.extract_info(data_str)
					else:
						rospy.logdebug("incoming_bytes_no = %d: %s",incoming_bytes_no, data_str)
			except Exception as e:
				connection_errors_no += 1
				if(connection_errors_no > 5):
					connection_errors_no = 0 #reset the counter
					self.reconnect_serial_port()
		rospy.logdebug("serial_port_reader_thread done.")

class states_publisher(threading.Thread):
	def __init__(self, loop_time):
		threading.Thread.__init__(self)
		self.loop_time = loop_time
		self.joint_state_msg = JointState()
		self.joint_state_msg.name = []
		self.joint_state_msg.name.append("gripper_claws")
		self.joint_states_publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
		self.updater = diagnostic_updater.Updater()
		self.updater.setHardwareID("Weiss Robotics Gripper IEG 76-030 V1.02 SN 000106")
		self.updater.add("Position and flags updater", self.produce_diagnostics)
		freq_bounds = {'min':0.5, 'max':2}
		# It publishes the messages and simultaneously makes diagnostics for the topic "joint_states" using a FrequencyStatus and TimeStampStatus
		self.pub_freq_time_diag = diagnostic_updater.DiagnosedPublisher(self.joint_states_publisher, self.updater, diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.1, 10), diagnostic_updater.TimeStampStatusParam())

	def run(self):
		global fresh_flags_cond_var
		while not shutdown_driver:
			fresh_flags_cond_var.acquire()
			fresh_flags_cond_var.wait()
			self.updater.update()
			self.publish_states()
			#self.updater.force_update()
			fresh_flags_cond_var.release()
			rospy.sleep(self.loop_time)
		rospy.logdebug("states_publisher_thread done.")

	def produce_diagnostics(self, stat):
		if FAULT_FLAG == True:
			stat.summary(DiagnosticStatus.ERROR, "The fault bit of the gripper is 1.")
		else:
			stat.summary(DiagnosticStatus.OK, "The fault bit of the gripper is 0.")
		stat.add("Position", POS)
		stat.add("Idle Flag", IDLE_FLAG)
		stat.add("Open Flag", OPEN_FLAG)
		stat.add("Closed Flag", CLOSED_FLAG)
		stat.add("Holding Flag", HOLDING_FLAG)
		stat.add("Error Flag", FAULT_FLAG)
		stat.add("Temperature Error Flag", TEMPFAULT_FLAG)
		stat.add("Temperature Warning Flag", TEMPWARN_FLAG)
		stat.add("Maintenance Flag", MAINT_FLAG)
		return stat

	def publish_states(self):
		self.joint_state_msg.header.stamp = rospy.Time.now()
		self.joint_state_msg.position = []
		self.joint_state_msg.position.append(POS)
		try:
			self.pub_freq_time_diag.publish(self.joint_state_msg)
		except:
			rospy.logerr("\nClosed topics.")

class weiss_gripper_ieg76(object):
	def __init__(self):
		global ser
		#rospy.init_node('weiss_gripper_ieg76_node', log_level=rospy.DEBUG)
		rospy.init_node('weiss_gripper_ieg76_node')
		rospy.on_shutdown(self.shutdown_handler)
		serial_port_addr = rospy.get_param("~serial_port_address", '/dev/ttyACM0')
		ser.port = serial_port_addr
		ser.timeout = 0
		is_serial_port_opened = False
		while not(shutdown_driver) and not(is_serial_port_opened):
			try: 
				ser.open()
				ser.flushInput()
				ser.flushOutput()
				is_serial_port_opened = True
			except Exception as e:
				is_serial_port_opened = False
				rospy.logerr("\terror opening serial port %s : %s", serial_port_addr, e)
				rospy.loginfo("Retrying to open the serial port %s...", serial_port_addr)
				time.sleep(1)
		
		rospy.loginfo("Serial port opened: %s", ser.isOpen())

		serv_ref = rospy.Service('reference', Trigger, self.handle_reference)
		serv_open = rospy.Service('open_jaws', Trigger, self.handle_open_jaws)
		serv_close = rospy.Service('close_jaws', Trigger, self.handle_close_jaws)
		serv_grasp = rospy.Service('grasp_object', Trigger, self.handle_grasp_object)
		serv_ack_error = rospy.Service('ack_error', Trigger, self.handle_ack_error)
		serv_ack_ref_error = rospy.Service('ack_ref_error', Trigger, self.handle_ack_ref_error)

		self.serial_port_reader_thread = serial_port_reader()
		self.states_publisher_thread = states_publisher(0.8)

		self.initialize_gripper()

		rospy.loginfo("Ready to receive requests.")

	def initialize_gripper(self):
		rospy.loginfo("Query")
		payload = create_send_payload("query")
		serial_port_lock.acquire()
		try:
			ser.write(payload)
			time.sleep(0.5)
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Fallback")
			payload = create_send_payload("fallback")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Mode")
			payload = create_send_payload("mode")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Restart")
			payload = create_send_payload("restart")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Operate")
			payload = create_send_payload("operate")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Reset flags")
			payload = create_send_payload("reset")
			ser.write(payload)
			time.sleep(0.5)
		
		except Exception as e:
			rospy.logerr("initialize_gripper: %s", e)
		finally:
			serial_port_lock.release()

	def handle_reference(self, req):
		rospy.loginfo("Referencing")
		payload = create_send_payload("reference")
		reply = TriggerResponse()

		reference_cond_var.acquire()

		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
			except SerialException as e:
				rospy.logerr("Error writing to the serial port: %s", e)
			finally:
				serial_port_lock.release()
				
				reference_cond_var.wait(timeout=3.0)
				if IDLE_FLAG:
					rospy.loginfo("Gripper referenced.")
					reply.success = True
					reply.message = "Gripper referenced."
				else:
					rospy.logerr("Failed to reference the gripper.")
					reply.success = False
					reply.message = "Failed to reference the gripper."
		except Exception as e:
			rospy.logerr("weiss_gripper_ieg76.handle_reference(): %s", e)
		finally:
			log_debug_flags()
			reference_cond_var.release()

		return reply

	def handle_open_jaws(self, req):
		rospy.loginfo("Opening the jaws.")
		payload = create_send_payload("open")
		reply = TriggerResponse()

		jaws_opened_cond_var.acquire()
		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
				rospy.logdebug("Message sent to serial port")
			except SerialException as e:
				rospy.logerr("Error writing to the serial port: %s", e)
			finally:
				rospy.logdebug("After sending the message to the serial port")
				serial_port_lock.release()	
				
			jaws_opened_cond_var.wait(timeout=3.0)
			if OPEN_FLAG:
				rospy.loginfo("Jaws opened.")
				reply.success = True
				reply.message = "Jaws opened."
			else:
				rospy.logerr("Failed to open the jaws.")
				reply.success = False
				reply.message = "Failed to open the jaws."
				if HOLDING_FLAG:
					rospy.logwarn("Remove the object which is blocking the claws from opening completly and try again.")
					reply.message = reply.message + " Remove the object which is blocking the claws from opening completly and try again."
		except Exception as e:
			rospy.logerr("weiss_gripper_ieg76.handle_open_jaws(): %s", e)				
		finally:
			log_debug_flags()
			jaws_opened_cond_var.release()		

		return reply

	def handle_close_jaws(self, req):
		rospy.loginfo("Completly closing the jaws.")
		payload = create_send_payload("close")
		reply = TriggerResponse()

		jaws_closed_cond_var.acquire()
		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
			except SerialException as e:
				rospy.logerr("weiss_gripper_ieg76.handle_close_jaws() while trying to write on the serial port: %s", e)	
			finally:
				serial_port_lock.release()	

			jaws_closed_cond_var.wait(timeout=3.0)#always returns None for python 2
			if CLOSED_FLAG:
				rospy.loginfo("Jaws completly closed.")
				reply.success = True
				reply.message = "Jaws completly closed."	
			else:
				rospy.logerr("Failed to completly close the jaws.")
				reply.success = False
				reply.message = "Failed to completly close the jaws."

		except Exception as e:
			rospy.logerr("weiss_gripper_ieg76.handle_close_jaws(): %s", e)
		finally:
			log_debug_flags()
			jaws_closed_cond_var.release()

		return reply

	def handle_grasp_object(self, req):
		rospy.loginfo("Grasping object.")
		payload = create_send_payload("close")
		reply = TriggerResponse()

		object_grasped_cond_var.acquire()
		try:
			log_debug_flags()

			try:
				serial_port_lock.acquire()
				ser.write(payload)
			except SerialException as e:
				rospy.logerr("Error while writing on the serial port: %s", e)	
			finally:
				serial_port_lock.release()		

				object_grasped_cond_var.wait(timeout=3.0)
				if HOLDING_FLAG:
					rospy.loginfo("Object grasped.")
					reply.success = True
					reply.message = "Object grasped."
				else:
					rospy.logerr("Timed out while trying to grasp an object.")
					reply.success = False
					reply.message = "Timed out while trying to grasp an object."
					if CLOSED_FLAG: 
						rospy.logwarn("No object to grasp.")
						reply.message = reply.message + " No object to grasp."
		except Exception as e:
			rospy.logerr("weiss_gripper_ieg76.handle_grasp_jaws(): %s", e)				
		finally:
			log_debug_flags()
			object_grasped_cond_var.release()		

		return reply

	def handle_ack_error(self, req):
		rospy.loginfo("Acknowledging error")
		payload = create_send_payload("activate")
		reply = TriggerResponse()

		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
				time.sleep(0.5)
				payload = create_send_payload("deactivate")
				ser.write(payload)
				time.sleep(1)
			except SerialException as e:
				rospy.logerr("Error writing to the serial port: %s", e)
			finally:
				serial_port_lock.release()
				
				fault_cond_var.acquire()
				if not FAULT_FLAG:
					rospy.loginfo("Error acknowledged.")
					reply.success = True
					reply.message = "Error acknowledged"
				else:
					rospy.logerr("Failed to acknowledge the error.")
					reply.success = False
					reply.message = "Failed to acknowledge the error"
		except Exception as e:
			rospy.logerr("weiss_gripper_ieg76.handle_ack_error(): %s", e)
		finally:
			log_debug_flags()
			fault_cond_var.release()

		return reply

	def handle_ack_ref_error(self, req):
		rospy.loginfo("Acknowledging reference error")
		payload = create_send_payload("deactivate")
		reply = TriggerResponse()

		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
				time.sleep(1)
			except SerialException as e:
				rospy.logerr("Error writing to the serial port: %s", e)
			finally:
				serial_port_lock.release()
				
				fault_cond_var.acquire()
				if not FAULT_FLAG:
					rospy.loginfo("Reference error acknowledged.")
					reply.success = True
					reply.message = "Reference error acknowledged"
				else:
					rospy.logerr("Failed to acknowledge the reference error.")
					reply.success = False
					reply.message = "Failed to acknowledge the reference error"
		except Exception as e:
			rospy.logerr("weiss_gripper_ieg76.handle_ack_ref_error(): %s", e)
		finally:
			log_debug_flags()
			fault_cond_var.release()

		return reply

	def shutdown_handler(self):
		global shutdown_driver
		shutdown_driver = True
		payload = create_send_payload("deactivate")

		try:
			rospy.loginfo("Deactivate.")
			serial_port_lock.acquire()
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Fallback.")
			payload = create_send_payload("fallback")
			ser.write(payload)
			time.sleep(0.5)
			if ser.isOpen():
				rospy.loginfo("Close port.")
				ser.close()
		except SerialException as e:
			rospy.logerr("Error closing the serial port: %s", e)
		finally:
			serial_port_lock.release()
		rospy.loginfo("Gracefully shutting down the driver...")

	def run(self):
		self.serial_port_reader_thread.daemon = True
		self.states_publisher_thread.daemon = True

		rospy.loginfo("Starting threads...")
		self.serial_port_reader_thread.start()
		self.states_publisher_thread.start()
		rospy.loginfo("Threads started.")
		
		rospy.spin()

if __name__ == "__main__":
	driver = weiss_gripper_ieg76()
	driver.run()