import serial
from pyubx2 import UBXReader
ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
setNav = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC]

gps = {}
while True:
	ser.write(bytes(setNav))
	'''
	ack = ser.read(2000)
	if b'\xb5b' in ack:
		print("GPS entered flight mode successfully")
	else:
		print("GPS failed to enter flight mode")
	'''
	ubr = UBXReader(ser)
	msg = (ubr.read()[1])
	print(msg)
	if "GNGGA" in str(msg):
		gps["time"] = str(msg).split(",")[1].replace("time=", "")
		gps["lat"] = str(msg).split(",")[2].replace("lat=", "")
		gps["lon"] = str(msg).split(",")[4].replace("lon=", "")
		gps["alt"] = str(msg).split(",")[9].replace("alt=", "")
		print(gps)
	if "GNRMC" in str(msg):
		gps["speed"] = str(msg).split(",")[7].replace("spd=", "")
		print(gps)

