import time
import serial

g = 0
ser = serial.Serial(
        port='/dev/ttyS0',
        baudrate = 38400,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)

while 1:
        x=ser.readline()
        if(x != 0):
            g = g+1;
            
        
        if(g == 50):
            print (x)
            g = 0;
#         $GPGSV
#         $GNGSA
#         $GNRMC
#         $GLGSV