pin = 19
import pigpio
import time
pi = pigpio.pi()
pi.set_servo_pulsewidth(pin, 1000)
time.sleep(5)
pi.set_servo_pulsewidth(pin, 1000)
time.sleep(1)
pi.set_servo_pulsewidth(pin, 1500)

time.sleep(1)
pi.set_servo_pulsewidth(pin, 2000)

time.sleep(1)
pi.set_servo_pulsewidth(pin, 1000)