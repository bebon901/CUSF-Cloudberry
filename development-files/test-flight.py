#from pyubx2 import *
import pyubx2
import serial
from pyubx2 import UBXReader, UBXMessage, SET

#print(pyubx2.UBX_MSGIDS)
ser = serial.Serial("/dev/serial0", baudrate=9600)

# request message to get the navigation configuration 
msg = UBXMessage("CFG", "CFG-NAV5", SET, msgClass=0x06, msgID=0x24, dynModel=0x06)
print(msg)

#mssasag = UBXReader.parse(b'\xb5b\0xB5\0x62\0x06\0x24\0x24\0x00\0xFF\0xFF\0x06\0x03\0x00\0x00\0x00\0x00\0x10\0x27\0x00\0x00\0x05\0x00\0xFA\0x00\0xFA\0x00\0x64\0x00\0x2C\0x01\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x16\0xDC', msgmode=SET)

# send the message to the GPS
ser.write(msg.serialize())

# read the response from the GPS
for i in range(20):
	response = ser.readline()
	print(response)

	# check if the GPS is in airborne <1g mode 
	# by checking the value of the "dynModel" field in the response payload
	dynModel = response[4]
	if dynModel == 6:
		print("The GPS is in airborne <1g mode")
	else:
		print("The GPS is not in airborne <1g mode")
