import smbus #comunicación I2C
from time import sleep
import math

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
    if a[len(a)-1] == 0:
        bandera = bus.read_byte_data(deviceAdress,RegStatus)
        x=read_raw_data(eje_X_Mag)
        y=read_raw_data(eje_Y_Mag)
        z=read_raw_data(eje_Z_Mag)
        heading = math.atan2(y,x)+ declination
    #compensar superiores a 360
    if(heading > 2*pi):
        heading = heding -2*pi
    #revisar el signo
    if(heading < 0):
        heading=heading+2*pi
    #convertir a grados
    heading_angle = int(heading * (180/pi))
    print("angulo = %d°" %heading_angle)
    sleep(0.5)