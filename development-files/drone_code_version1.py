import pigpio
import time
import board
import adafruit_lsm6ds.lsm6dso32

import radio
import serial
ser = serial.Serial("/dev/serial0", 115200, timeout=1) ##, parity="N", stopbits=1, bytesize=8)
print("Serial Port Opened")

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_lsm6ds.lsm6dso32.LSM6DSO32(i2c)
valued = radio.values


P = 0.2

def constrain(value, min_val, max_val):
    if value <= min_val:
        return min_val
    if value >= max_val:
        return max_val
    return value
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
    
    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.last_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output
    


roll = PIDController(kp=P, ki=0, kd=0.0)
pitch = PIDController(kp=P, ki=0, kd=0.0)
pi = pigpio.pi()
motor1 = 26
motor2 = 13
motor3 = 6
motor4 = 19
pi.set_servo_pulsewidth(motor1, 1000)
pi.set_servo_pulsewidth(motor2, 1000)
pi.set_servo_pulsewidth(motor3, 1000)
pi.set_servo_pulsewidth(motor4, 1000)
time.sleep(5)



flat_x, flat_y, flat_z = sensor.acceleration 
while True:
    valued.data = ser.read(16)
    input_value = radio.parse_bytes(valued.data)
    
    acc_x, acc_y, acc_z = sensor.acceleration
    #print(flat_y, acc_y)
    #print(roll.compute(flat_y, acc_y))
    print(input_value)
    m1_roll_pw = -roll.compute(flat_y, acc_y) / 10 * 1000
    m2_roll_pw = roll.compute(flat_y, acc_y) / 10 * 1000
    m3_roll_pw = roll.compute(flat_y, acc_y) / 10 * 1000
    m4_roll_pw = -roll.compute(flat_y, acc_y) / 10 * 1000
    
    m1_pitch_pw = -pitch.compute(flat_x, acc_x) / 10 * 1000
    m2_pitch_pw = -pitch.compute(flat_x, acc_x) / 10 * 1000
    m3_pitch_pw =  pitch.compute(flat_x, acc_x) / 10 * 1000
    m4_pitch_pw =  pitch.compute(flat_x, acc_x) / 10 * 1000
    
    
    pi.set_servo_pulsewidth(motor1, constrain((input_value["Throttle"]-406) / 1024 * 1000 + m1_roll_pw + m1_pitch_pw + 1000, 1000, 2500))
    
    pi.set_servo_pulsewidth(motor2, constrain((input_value["Throttle"]-406) / 1024 * 1000 + m2_roll_pw + m2_pitch_pw + 1000, 1000, 2500))
    
    pi.set_servo_pulsewidth(motor3, constrain((input_value["Throttle"]-406) / 1024 * 1000 + m3_roll_pw + m3_pitch_pw + 1000, 1000, 2500))
    
    pi.set_servo_pulsewidth(motor4, constrain((input_value["Throttle"]-406) / 1024 * 1000 + m4_roll_pw + m4_pitch_pw + 1000, 1000, 2500))
