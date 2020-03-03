#rpi_serial_test

#GOALS
#1. send whole word
#2. receive whole word

import serial
import time

body = {'motor_L':0, 'motor_R':0}
arm = {'base':0, 'shoulder':0, 'elbow':0, 'wrist':'H', 'gripper':'closed'}
cmd_group = {'cmd1':1, 'cmd2':2}

com = serial.Serial('/dev/serial0')

def set_cmd ():
	global cmd_group
	cmd_group = {'mov_fwd':50}

def send_cmd ():
	global com
	for cmd in cmd_group:
		cmd_name = list(cmd)
		for ltr in cmd_name:
			com.write(ltr)
		com.write(':')
		com.write(cmd_group[cmd])


def send_serial():
	ser = serial.Serial('/dev/serial0')
	while True:
		ser.write('Run'.encode())
		time.sleep(5)
		if (ser.read()):
			message = ser.read()
			print('message: ')
			print(message)

set_cmd()
send_cmd()
