#rpi_serial_send_codes

#GOALS
#1. send whole word
#2. receive whole word

import serial
import time
import struct

#robot
body = {'motor_l':0.0, 'motor_r':0.0, 'encoder_l':0.0, 'encoder_r':0.0}     #motor speeds, encoder values
arm = {'base':0.0, 'shoulder':0.0, 'elbow':0.0, 'wrist':0.0, 'gripper':0.0}   #encoder values, orientation (wrist:0=horizontal, 1=vertical, gripper:0=closed, 1= open)

#command codes
cmd_codes = {"mov_f":10, "mov_b":15, "turn_l":20, "turn_r":25, "spin_l":30, "spin_r":35, "base":40, "shoulder":50, "elbow":60, "wrist":70, "gripper":80}

#default command group
cmd_group = {'1':1.0, '2':2.0}

port = serial.Serial('/dev/serial0')	#serial port address for Serial2 on MegaPi

def set_cmd ():
	global cmd_group
	cmd_group = {'10':100.0, '15':200.0, '40':300.0, '50':500.0}        #enter values as floats to use the period as a natural delimiter between commands,

def send_cmd ():
	global port
	for cmd in cmd_group:
		cmd_code = list(cmd)
		print(cmd_code)
		print(cmd_group[cmd])
		for digit in cmd_code:
			port.write(digit.encode())
		port.write(':'.encode())
		cmd_value = str(cmd_group[cmd])
		for digit in cmd_value:
			port.write(digit.encode())

set_cmd()
send_cmd()
