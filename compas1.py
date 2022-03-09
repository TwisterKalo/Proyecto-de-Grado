import time
import py_hmc5883l
sensor = py_hmc5883l.HMC5883L()

while True:
    print sensor.get_magnet()
    time.sleep(0.2)