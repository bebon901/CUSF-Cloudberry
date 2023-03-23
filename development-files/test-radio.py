import radio
import serial
ser = serial.Serial("/dev/serial0", 115200, timeout=1) ##, parity="N", stopbits=1, bytesize=8)
print("Serial Port Opened")
valued = radio.values
for y in range(10):
	valued.data = ser.read(16)
	print(radio.parse_bytes(valued.data)["Throttle"])

	#print(valued.data)
	#print(valued.throttle)
'''
radiovals = radio.value()
print(radiovals.throttle)
radiovals.update()
print(radiovals.throttle)
'''