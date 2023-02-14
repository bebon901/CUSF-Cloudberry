import serial
ser = serial.Serial("/dev/serial0", 115200)
while True:
	print(ser.readline())
