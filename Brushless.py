#librerias0
import serial
import numpy as np
import math as mt
from time import sleep
import RPi.GPIO as GPIO
import smbus #comunicación I2C
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

def servo(ang):

    x = ang
    x = min(max(0,x),180)
    print(x)
    kit.servo[14].angle = x
def servo1(ang):

    x = ang
    x = min(max(0,x),180)
    print(x)
    kit.servo[13].angle = x
        
while True:
    x = float(input("velocidad del motor     "))
    y = float(input("angulo del motor     "))
    servo(x)
    servo1(y)
    
#     g = kit.servo[14].angle       
#     servo(0)
#     sleep(30

    
    

#     print(g)
#     servo(6)
#     sleep(3)
#     servo(0)
#     sleep(30)
# Max Throttle (2000/100)
# Min Throttle (700/~60)
# Stop Throttle (0)

# https://content.instructables.com/ORIG/F3N/FP21/J0MOOXHS/F3NFP21J0MOOXHS.py