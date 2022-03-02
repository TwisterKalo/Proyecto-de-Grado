#librerias
import serial
import numpy as np
import math as mt
from time import sleep
import RPi.GPIO as GPIO

#configuracion de sistema
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(32,GPIO.OUT)
GPIO.setup(33,GPIO.OUT)

servo = GPIO.PWM(32,ang)
servo.start(0)

#variables
#posicion del GPS
pos = np.zeros(2)

#funciones
#leer Gps
def leer_gps(pos):
    try:
        gps = serial.Serial(
        port='/dev/ttyS0',
        baudrate = 38400,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
        data = "0"

        while data[0] != '$GNRMC':
            ser_bytes = gps.readline()
            decoded_bytes = ser_bytes.decode("utf-8")
            data = decoded_bytes.split(",")
                
        if data[0] == '$GNRMC':
            lat_nmea = data[3]
            lat_grad = lat_nmea[:2]
            lat_ddd = lat_nmea[2:10]
            lat_mmm = float(lat_ddd) / 60
                    
            if data[6] == 'S':
                latitud_grad = float(lat_grad) * -1
                latitud = latitud_grad - lat_mmm

            else:
                latitud_grad = float(lat_grad)
                latitud = latitud_grad + lat_mmm

                        
            long_nmea = data[5]
            long_grad = long_nmea[1:3]
            long_ddd = long_nmea[3:10]
            long_mmm = float(long_ddd) / 60
                    
            if data[6] == 'W':
                longitud_grad = float(long_grad) * -1
                longitud = longitud_grad - long_mmm

            else:
                longitud_grad = float(long_grad)
                longitud = longitud_grad + long_mmm


            pos[0] = float(longitud)
            pos[1] = float(latitud)

            return pos

                    
    except serial.SerialException:
        print("Fallo en comunicacion con el GPS!")

def mov_Servo(ang):
    servo.ChangeDutyCycle(7.5) # neutral
    sleep(1)


while True:
    #pos = leer_gps(pos)
    x = float(input("cual es el angulo?: "))
    print(x)
    mov_Servo(x)
    sleep(2)