import radio
import serial
import time
import board
import busio
import pigpio
import radio
import serial
ser = serial.Serial("/dev/serial0", 115200, timeout=1) ##, parity="N", stopbits=1, bytesize=8)
print("Serial Port Opened")

pi = pigpio.pi()


def map_range(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

valued = radio.values
stick_scaling = 0.6
keys = [["Throttle", 26, 1], ["Pitch", 13, 1], ["Roll", 6, -1], ["Yaw", 19, -1]]

maxpul = 2200
minpul = 700
while True:
    valued.data = ser.read(16)
    input_value = radio.parse_bytes(valued.data)
    #print(input_value)
    for i in range(len(keys)):
        #print((map_range(input_value[keys[i][0]], 400, 1700, minpul, maxpul)))
        if keys[i][2] == 1:
            pi.set_servo_pulsewidth(keys[i][1], (map_range(input_value[keys[i][0]], 400, 1700, minpul, maxpul)))
        if keys[i][2] == -1:
            pi.set_servo_pulsewidth(keys[i][1], (map_range(input_value[keys[i][0]], 1700, 400, minpul, maxpul)))
