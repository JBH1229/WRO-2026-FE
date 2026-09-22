import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep
import time
import serial
from gpiozero import Button

button = Button(5)
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
arduino.write_timeout = 1.0
# Most Arduinos reset when the port opens; give it time to boot
time.sleep(2.0)
# Clear anything the Arduino might have printed during boot
servo_value = 66
motor_value = 1500
#set default values
arduino.reset_input_buffer()
arduino.reset_output_buffer()

def send_servo(value):
    global servo_value
    value = int(value)
    if value < 0:
        value = 0
    if value > 180:
        value = 180
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@S%b\n" %valueb)
    servo_value = value

#@log
def send_motor(value):
    global motor_value
    value = int(value)
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@M%b\n" %valueb)
    motor_value = value
#@log
def send_servo_assigned(value): 
    global servo_value
    value = int(value)
    value = 66 + value
    if value < 0:
        value = 0
    if value > 180:
        value = 180
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@S%b\n" %valueb)
    servo_value = value
#@log
def send_motor_assigned(value):
    global motor_value
    value = int(value)
    value = 1500 + value
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@M%b\n" %valueb)
    motor_value = value
#@log
def send_servo_relative(value): 
    global servo_value
    value = int(value)
    value = servo_value + value
    if value < 0:
        value = 0
    if value > 180:
        value = 180
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@S%b\n" %valueb)
    servo_value = value
#@log
def send_motor_relative(value):
    global motor_value
    value = int(value)
    value = motor_value + value
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@M%b\n" %valueb)
    motor_value = value

#start moving (motor start)
print("waiting for button press...")
#button.wait_for_press()
send_servo(82)
"""
send_servo(30)
sleep(0.75)
send_motor(1500)
send_servo(82)
sleep(0.5)
send_servo(120)
send_motor(1390)
sleep(0.5)
send_motor(1500)
send_servo(82)
sleep(0.5)
"""
send_servo(120)
send_motor(1620)
sleep(0.5)
send_motor(1500)
send_servo(82)
sleep(0.5)
send_motor(1620)
sleep(0.5)
send_motor(1500)
send_servo(82)
sleep(0.5)
send_servo(30)
send_motor(1620)
sleep(0.5)
send_motor(1500)
send_servo(82)
sleep(0.5)


