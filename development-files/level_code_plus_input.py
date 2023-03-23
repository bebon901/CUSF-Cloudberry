import time
import board
import busio
import adafruit_lsm6ds.lsm6dso32
import pigpio
import math
import board

import radio
import serial
ser = serial.Serial("/dev/serial0", 115200, timeout=1) ##, parity="N", stopbits=1, bytesize=8)
print("Serial Port Opened")


# Initialize the accelerometer and gyroscope
i2c = board.I2C()
sensor = adafruit_lsm6ds.lsm6dso32.LSM6DSO32(i2c)

# Initialize the PID controller
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

pid = PIDController(kp=2, ki=0.1, kd=0.1)
pi = pigpio.pi()
class servo:
    def __init__(self, pin, angle=90):
         self._angle=angle
         self._pin=pin
    @property
    def angle(self):
      return self._angle
    @angle.setter
    def angle(self, value):
      if self._angle < -90:
        self._angle = -90
      if self._angle > 90:
        self._angle = 90
      self._angle = value
      pi.set_servo_pulsewidth(self._pin, (self._angle/180)*(2000-1000)+1000)
# Set the initial servo positions
elevator = servo(pin=13, angle=90)
wing_left = servo(pin=26)
wing_right = servo(pin=19)
elevator.angle = 90
wing_left.angle = 0
wing_right.angle = 180
gyro_angle = 0
angle = 0
# Main loop

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
while True:
    valued.data = ser.read(16)
    input_value = radio.parse_bytes(valued.data)["Roll"]
    input_angle = map_range(input_value, 500, 1548, -90, 90)
    #print(input_value)
    #print(input_angle)
    gyro_x, gyro_y, gyro_z = sensor.gyro
    accel_x, accel_y, accel_z = sensor.acceleration

    # calculate gyro angle
    gyro_angle += gyro_x * 0.01 # 0.01 is the time between each loop iteration
    #gyro_angle = constrain(gyro_angle, -90, 90)  # constrain gyro angle to between -90 and 90 degrees

    # calculate acceleration angle
    accel_angle = math.atan2(accel_y, accel_z) * 180 / math.pi
    #accel_angle = constrain(accel_angle, -90, 90)  # constrain acceleration angle to between -90 and 90 degrees

    # combine gyro and acceleration angles using a complementary filter
    angle = 0 * (angle + gyro_angle) + 1* accel_angle

    #elevator.angle = pulsewidth
    wing_left.angle = angle + input_angle*stick_scaling  # adjust offsets as needed
    wing_right.angle = angle - input_angle*stick_scaling  # adjust offsets as needed

    # Wait for a short amount of time to avoid overloading the servos
    #time.sleep(0.01)

