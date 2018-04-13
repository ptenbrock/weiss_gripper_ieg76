#!/usr/bin/env python
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
from transitions.extensions import LockedHierarchicalMachine as Machine


class DriverLogic(object):
	states = [
		'not_initialized', 'fault', 'other_fault',
		'inactive', 'open', 'closed', 'holding',
		{ 'name': 'referencing', 'on_enter': 'exec_referencing', 'on_exit': 'operation_finished'},
		{ 'name': 'opening', 'on_exit': 'operation_finished',
		  'children':[{'name': 'preclosing', 'on_enter': 'exec_preclosing'},
		  			  {'name': 'opening', 'on_enter': 'exec_opening'}]},
		{ 'name': 'closing', 'on_exit': 'operation_finished',
		  'children':[{'name': 'preopening', 'on_enter': 'exec_preopening'},
		  			  {'name': 'closing', 'on_enter': 'exec_closing'}]},
		{ 'name': 'grasping', 'on_exit': 'operation_finished',
		  'children':[{'name': 'preopening', 'on_enter': 'exec_preopening'},
		  			  {'name': 'grasping', 'on_enter': 'exec_grasping'}]}
	]

	transitions = [
		# uninitialized, fault and other_fault state
		['on_inactive', ['not_initialized', 'fault', 'other_fault'], 'inactive'],
		['on_open', ['not_initialized', 'fault', 'other_fault'], 'open'],
		['on_closed', ['not_initialized', 'fault', 'other_fault'], 'closed'],
		['on_holding', ['not_initialized', 'fault', 'other_fault'], 'holding'],

		# inactive state
		{ 'trigger': 'do_open', 'source': 'inactive', 'dest': 'opening', 'before': 'exec_activate'},
		{ 'trigger': 'do_close', 'source': 'inactive', 'dest': 'closing', 'before': 'exec_activate'},
		{ 'trigger': 'do_grasp', 'source': 'inactive', 'dest': 'grasping', 'before': 'exec_activate'},

		# open state
		['do_reference', 'open', 'referencing'],
		['do_open', 'open', 'closing_before_opening'],
		['do_close', 'open', 'closing'],
		['do_grasp', 'open', 'grasping'],

		# closed state
		['do_reference', 'closed', 'referencing'],
		['do_open', 'closed', 'opening'],
		['do_close', 'closed', 'opening_before_closing'],
		['do_grasp', 'closed', 'grasping'],

		# holding state
		['do_reference', 'holding', 'referencing'],
		{ 'trigger': 'do_open', 'source': 'holding', 'dest': 'opening', 'conditions': 'can_move_while_holding'},
		{ 'trigger': 'do_close', 'source': 'holding', 'dest': 'opening_before_closing', 'conditions': 'can_move_while_holding'},

		# referencing state
		['on_open', 'opening', 'open'],

		# opening state
		['on_open', 'opening', 'open'],

		# closing state
		['on_open', 'opening', 'open'],

		# grasping state
		['on_open', 'opening', 'open'],

		# closing_before_opening state
		['on_closed', 'closing_before_opening', 'opening'],

		# opening_before_closing state
		['on_open', 'opening_before_closing', 'closing'],
	]

	def __init__(self, serial_port_comm):
		self.machine = Machine(model=self, states=DriverModel.states, initial='not_initialized')
		self.serial_port_comm = serial_port_comm
		serial_port_comm.add_flags_observer(self)
		self.gripper_pos = None

		self.service_sync = threading.Lock()
		self.operation_params = None
		self.operation_response = None
		self.async_operation_finished = threading.Condition()
		self.no_spurious_wakeup = False

		self.old_flag_signaled = {"OPEN_SIGNALED":False, "CLOSED_SIGNALED":False, "HOLDING_SIGNALED":False,
								  "FAULT_SIGNALED":False, "IDLE_SIGNALED":False, "OTHER_FAULT_SIGNALED":False,
								  "NOT_INITIALIZED_SIGNALED":True}

	def service_called(self, transition, params, trigger_response):
		with self.service_sync: # only one service call allowed at a time
			self.no_spurious_wakeup = False
			self.operation_params = params
			self.operation_response = trigger_response
			with self.async_operation_finished: # needed if result of service known at later stage
				try:
					trigger_response.success = False # only successful if explicitly set
					self.trigger(transition)
					while not self.no_spurious_wakeup:
						self.async_operation_finished.wait() # needed if result of service known at later stage
				except transitions.core.MachineError as err: # not a valid action in the current state
					trigger_response.success = False
					trigger_response.message = str(err) + trigger_response.message
					return
				finally:
					if trigger_response.success:
						trigger_response.message = "Succeeded. " + trigger_response.message
					else:
						trigger_response.message = "Failed. " + trigger_response.message + " Current state: " + self.state
					self.operation_params = None
					self.operation_response = None

	def operation_finished(self):
		with self.async_operation_finished:
			self.no_spurious_wakeup = True
			self.async_operation_finished.notify()

	def operation_successful(self):
		self.operation_response.success = True

	def update_flags(self, new_flags_dict):
		self.gripper_pos = new_flags_dict["POS"]

		if new_flags_dict["TEMPFAULT_FLAG"] == 1 or new_flags_dict["MAINT_FLAG"]: #or new_flags_dict["TEMPWARN_FLAG"]:
			if not self.old_flag_signaled["OTHER_FAULT_SIGNALED"]:
				self.to_other_fault()
				self.old_flag_signaled["OTHER_FAULT_SIGNALED"] = True
		elif new_flags_dict["FAULT_FLAG"] == 1:
			if not self.old_flag_signaled["FAULT_SIGNALED"]:
				self.to_fault()
				self.old_flag_signaled["FAULT_SIGNALED"] = True
		elif new_flags_dict["IDLE_FLAG"] == 1:
			if not self.old_flag_signaled["IDLE_SIGNALED"]:
				self.on_inactive()
				self.old_flag_signaled["IDLE_SIGNALED"] = True
		elif new_flags_dict["OPEN_FLAG"] == 1:
			if not self.old_flag_signaled["OPEN_SIGNALED"]:
				self.on_open()
				self.old_flag_signaled["OPEN_SIGNALED"] = True
		elif new_flags_dict["CLOSED_FLAG"] == 1:
			if not self.old_flag_signaled["CLOSED_SIGNALED"]:
				self.on_closed()
				self.old_flag_signaled["CLOSED_SIGNALED"] = True
		elif new_flags_dict["HOLDING_FLAG"] == 1:
			if not self.old_flag_signaled["HOLDING_SIGNALED"]:
				self.on_holding()
				self.old_flag_signaled["HOLDING_SIGNALED"] = True
		elif not self.check_if_referenced(new_flags_dict):
			if not self.old_flag_signaled["NOT_INITIALIZED_SIGNALED"]:
				self.on_not_initialized()
				self.old_flag_signaled["NOT_INITIALIZED_SIGNALED"] = True

	def check_if_referenced(self, flags_dict):
		#when the gripper is not referenced all flags are 0
		gripper_referenced = False

		if (flags_dict["IDLE_FLAG"] == 1 or flags_dict["OPEN_FLAG"] == 1 or
				flags_dict["CLOSED_FLAG"] == 1 or flags_dict["HOLDING_FLAG"] == 1 or
				flags_dict["FAULT_FLAG"] == 1 or flags_dict["TEMPFAULT_FLAG"] == 1 or
				flags_dict["TEMPWARN_FLAG"] == 1 or flags_dict["MAINT_FLAG"] == 1):
			gripper_referenced = True

		return gripper_referenced

	def can_move_while_holding(self, event):
		pass

	def exec_activate(self, event):
		pass

	def exec_referencing(self, event):
		self.serial_port_comm.send_command_synced("reference")

	def exec_opening(self, event):
		pass

	def exec_closing(self, event):
		pass

	def exec_grasping(self, event):
		pass

	def exec_preopening(self, event):
		pass

	def exec_preclosing(self, event):
		pass
