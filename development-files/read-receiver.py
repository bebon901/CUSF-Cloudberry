import serial
ser = serial.Serial("/dev/serial0", 115200, timeout=1) ##, parity="N", stopbits=1, bytesize=8)
for y in range(10):
	data = ser.read(16)
	print(data)
