import time
import board
import busio
import adafruit_lsm6ds.lsm6dso32
import pigpio
import math
import board
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

pid = PIDController(kp=1, ki=0, kd=0.0)
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


while True:
    gyro_x, gyro_y, gyro_z = sensor.gyro
    accel_x, accel_y, accel_z = sensor.acceleration

    # calculate gyro angle
    gyro_angle += gyro_x * 0.01  # 0.01 is the time between each loop iteration
    #gyro_angle = constrain(gyro_angle, -90, 90)  # constrain gyro angle to between -90 and 90 degrees

    # calculate acceleration angle
    accel_angle = math.atan2(accel_y, accel_z) * 180 / math.pi
    #accel_angle = constrain(accel_angle, -90, 90)  # constrain acceleration angle to between -90 and 90 degrees

    # combine gyro and acceleration angles using a complementary filter
    angle = 0.98 * (angle + gyro_angle) + 0.02 * accel_angle

    # convert angle to servo pulse width
    pulsewidth = angle
    #print("looped")
    # set servo positions
    #elevator.angle = pulsewidth
    wing_left.angle = pulsewidth + 10  # adjust offsets as needed
    wing_right.angle = pulsewidth - 10  # adjust offsets as needed
    '''
    # Calculate the pitch and roll angles
    pitch = -180 * math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2)) / math.pi
    roll = 180 * math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) / math.pi
    
    # Apply the pitch and roll angles to the PID controller
    elevator_output = pid.compute(setpoint=0, measured_value=pitch)
    wing_output = pid.compute(setpoint=0, measured_value=roll)
    
    # Apply the PID output to the servos
    elevator.angle = 90 - elevator_output
    wing_left.angle = wing_output
    wing_right.angle = 180 - wing_output
    '''
    # Wait for a short amount of time to avoid overloading the servos
    time.sleep(0.01)
