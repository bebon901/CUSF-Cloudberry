import serial
ser = serial.Serial("/dev/serial0", 115200, timeout=1) ##, parity="N", stopbits=1, bytesize=8)
for y in range(10):
	data = ser.read(64)
	print(data)
	for i in range(len(data)):
		print(data[i])
		#print(int.from_bytes(data[i], "big"))

