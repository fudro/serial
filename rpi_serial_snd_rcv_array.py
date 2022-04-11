#rpi_serial_test

#Sends a byte array verial the serial port
#Receives a byte array via the serial port an displays the result.

import serial
import time
import binascii


def get_cmds ():

	pass

def send_cmd ():
	pass

def send_serial():
	ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
	cmd = bytearray([217, 67])
	while True:
		print("Sending Byte Array:")
		for char in cmd:
			print(char)
		ser.write(cmd)
		time.sleep(3)
		message = ser.readline()
		print("Receiving Byte Array:")
		#Display each received byte as a Number, Hex and Char
		for char in message:
			asciiValue = ord(char)
			cmd_elements = ["NUM", str(asciiValue), "HEX:", binascii.b2a_hex(char), "CHAR:", char]
			print('\t'.join(cmd_elements))
		print("\n")


send_serial()
