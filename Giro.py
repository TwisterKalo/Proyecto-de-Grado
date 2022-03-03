import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(32,GPIO.OUT)
GPIO.setup(33,GPIO.OUT)

f = GPIO.PWM(33,100)
servo = GPIO.PWM(32,50)
# servo.start(0)
f.start(0)
# def get_pwm(angle):
    
#     return(angle/18.0)+2.5
    
# servo.ChangeDutyCycle(2.5) # -90
# sleep(1)
# servo.ChangeDutyCycle(7.5) # neutral
# sleep(1)
# servo.ChangeDutyCycle(12.5) #+90
# sleep(1)
# servo.stop()

for x in range(30,40):
    f.ChangeDutyCycle(x)
    print(x)
    sleep(0.2)

f.stop(0)
