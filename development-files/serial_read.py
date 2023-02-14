import serial
from pyubx2 import UBXReader
ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
gps = {}
while True:
	ubr = UBXReader(ser)
	msg = (ubr.read()[1])
	#print(msg)
	if "GNGGA" in str(msg):
		gps["time"] = str(msg).split(",")[1].replace("time=", "")
		gps["lat"] = str(msg).split(",")[2].replace("lat=", "")
		gps["lon"] = str(msg).split(",")[4].replace("lon=", "")
		gps["alt"] = str(msg).split(",")[9].replace("alt=", "")
		print(gps)
	if "GNRMC" in str(msg):
		gps["speed"] = str(msg).split(",")[7].replace("spd=", "")
		print(gps)

