pin = 19
import pigpio
import time
pi = pigpio.pi()
pi.set_servo_pulsewidth(pin, 1500)
time.sleep(1)
pi.set_servo_pulsewidth(pin, 800)
