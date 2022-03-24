#librerias
import serial
import numpy as np
import math as mt
from time import sleep
import RPi.GPIO as GPIO
import smbus #comunicación I2C
from adafruit_servokit import ServoKit


#Inicializar pca
kit = ServoKit(channels=16)

#configuracion magnetometro
#direcciones requeridas
#---------------config
Registro_A = 0x0B
Registro_B = 0x09
RegStatus = 0x06
RegCtrl = 0x09
#---------------direcciones de la conexión
bus=smbus.SMBus(1)
deviceAdress = 0x0d
#---------------datos
eje_X_Mag = 0x00
eje_Y_Mag = 0x02
eje_Z_Mag = 0x04
declination = -0.00669
pi=3.14159265359

#configuracion de sistema
# GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
# GPIO.setup(32,GPIO.OUT)
# GPIO.setup(33,GPIO.OUT)

# f = GPIO.PWM(33,100)
# f.start(0)

# servo = GPIO.PWM(32,50)
# servo.start(0)

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
#leer compas
def leer_compas():
    def MagnetometerInit():
    #configurar registro A
        bus.write_byte_data(deviceAdress, Registro_A, 0x01)
    #configurar registro B
        bus.write_byte_data(deviceAdress, Registro_B, 0x1D)
    #configurar registro para seleccionar el modo
    #bus.write_byte_data(deviceAdress, ModoRegistro, 0)

    def read_raw_data(addr):
    #leer doble byte (16 bits)
        low = bus.read_byte_data(deviceAdress,addr)
        high = bus.read_byte_data(deviceAdress,addr+1)
    #concatenar los bytes
        valor = ((high << 8) | low)
    #obtener el signo
        if(valor > 32768):
            valor = valor - 65536
        return valor
    #------------------------------------------main
    MagnetometerInit()
    print('leyendo magnetometro...')
    while True:
        bandera = bus.read_byte_data(deviceAdress,RegStatus)
        a="{0:b}".format(bandera)
        if int(a[len(a)-2]) == 0:
            bandera = bus.read_byte_data(deviceAdress,RegStatus)
            x=read_raw_data(eje_X_Mag)
            y=read_raw_data(eje_Y_Mag)
            z=read_raw_data(eje_Z_Mag)
            heading = mt.atan2(y,x)+ declination
        #compensar superiores a 360
        if(heading > 2*pi):
            heading = heding -2*pi
        #revisar el signo
        if(heading < 0):
            heading=heading+2*pi
        #convertir a grados
        heading_angle = int(heading * (180/pi)) - 254
        print("angulo = %d°" %heading_angle)
        sleep(0.5)
        return heading_angle
        
#mover el servo
def mov_Servo(ang):
    ang = (ang/18.0)+2.5
    servo.ChangeDutyCycle(ang) # neutral
    sleep(1)
#mover el motor
def mov_mo(v):
    print(v)
    f.ChangeDutyCycle(v)
#calcular el angulo de rotacion del robot
def angulo(target, pos1):
    #ang debe ser la orientacion del robot leida por los nodos del magnetometro
    ang = leer_compas()

    ang = (ang*mt.pi)/180

    ubicacion = leer_gps(pos1)
    #obj es la matriz homogenea para 2 dimenciones (x,y) teniendo el giro en Z 
    #obj es la postura del robot

    obj = np.array([[mt.cos(ang),-mt.sin(ang),ubicacion[0]],[mt.sin(ang),mt.cos(ang),ubicacion[1]],[0,0,1]])  # ([[-mt.sin(ang),mt.cos(ang),ubicacion[1]],[mt.cos(ang),mt.sin(ang),ubicacion[0]],[0,0,1]])

    obj = np.linalg.inv(obj)
    
    #Target (seria bueno que se le pregunte al usuario el target de manera manual)
    tar = np.array([[target[0]],[target[1]],[1]])


    #Calcular la posicion del target con respecto al robot
    

    pos = obj @ tar


    #print("La posicion del target con respecto al robot es: (x: %s , y: %s) " %(pos[0,0],pos[1,0]))

    #Se calcula el angulo de error (angulo que necesitamos rotar)
    ang_gi_rad = mt.atan2(pos[0,0],pos[1,0])
    ang_gi = ang_gi_rad* (180/3.14)

    return ang_gi, ubicacion
#mover el servo
def servo(ang):

    x = ang + 90
    x = min(max(0,x),180)
    kit.servo[13].angle = x
    
#inicio del programa
x = float(input("cual es la longitud?:  "))
y = float(input("cual es la latitud?:  "))

target = np.array([x,y])

#while True:
#    ang, pos = angulo(target,pos)

#    servo(ang)
#    print(ang)

while True:
    servo(0)
    sleep(2)
    servo(90)
    sleep(2)