import numpy as np
import math as mt
from time import sleep
# longitud: -69.96621
# latitud: 18.46796133

class Bote:
    def seek(self):
        force = target - pos
        
#         ang = mt.atan((force[1])/(force[0]))
#         ang = ang * (180/mt.pi)
#         print(ang)
        
        mag = np.linalg.norm(force)
        force = (force / mag) * max_speed # Set magnitud de max_speed
        force = force - vel  # steering
        
        # limitar el maximo del steering
        mag_force = np.linalg.norm(force)
        
        if (mag_force > max_force):
            force = (force / mag_force) * max_force
        
        self.applyForce(force)
    
    self.pos = np.array([18.46796133,-69.96621])
    self.vel = np.array([0,0])
    self.acc = np.array([0,0])
    self.max_speed = 4 # magnitud
    self.max_force = 0.25 #magnitud
    force = 0
    target = np.array([18.47068,-69.96790])
        
        
        
        
        
#     def applyForce(force):
#         print(acc)
#         self.acc = acc + force
#         print(acc)
#         sleep(0.5)
#     def update(vel, acc):
#         print(acc)
#         vel = vel + acc
#         # limitar el maximo de velocidad
#         mag_vel = np.linalg.norm(vel)
# 
#         if (mag_vel > max_speed):
#             vel = (vel / mag_vel) * max_speed
#             
#     update()
    
    seek()