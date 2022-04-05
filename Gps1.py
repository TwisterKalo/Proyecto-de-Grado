import serial
import numpy as np
import math as mt
from time import sleep


vel = np.array([0,0])
acc = np.array([0,0])
max_speed = 4
max_force = 0.25
force = 0
pos = np.zeros(2)
a = 0
i = 0

try:
        gps = serial.Serial(
        port='/dev/ttyS0',
        baudrate = 38400,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)
        
        while True:
            ser_bytes = gps.readline()
            decoded_bytes = ser_bytes.decode("utf-8")
            data = decoded_bytes.split(",")
#             print(data)
#             print(data[0])
            
            if data[0] == '$GNRMC':
                lat_nmea = data[3]
                lat_grad = lat_nmea[:2]
                lat_ddd = lat_nmea[2:10]
                lat_mmm = float(lat_ddd) / 60
                
                if data[6] == 'S':
                    latitud_grad = float(lat_grad) * -1
                    latitud = latitud_grad - lat_mmm
#                     print(latitud)
                else:
                    latitud_grad = float(lat_grad)
                    latitud = latitud_grad + lat_mmm
#                     print(latitud)
                    
                long_nmea = data[5]
                long_grad = long_nmea[1:3]
                long_ddd = long_nmea[3:10]
                long_mmm = float(long_ddd) / 60
                
                if data[6] == 'W':
                    longitud_grad = float(long_grad) * -1
                    longitud = longitud_grad - long_mmm
#                     print(longitud)
                else:
                    longitud_grad = float(long_grad)
                    longitud = longitud_grad + long_mmm
#                     print(longitud)

                pos[0] = float(longitud)
                pos[1] = float(latitud)
                print(pos)
                
except serial.SerialException:
    print("No hay GPS")
    
    #             while True:
#                 try:
#                     decoded_bytes = ser_bytes.decode("utf-8")
#                     break
#                 except UnicodeDecodeError:
#                     pass
#             print("listo")