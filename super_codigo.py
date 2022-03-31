#librerias
import serial
import numpy as np
import math as mt
from time import sleep
import RPi.GPIO as GPIO
import smbus #comunicación I2C
from adafruit_servokit import ServoKit
import os
import argparse
import cv2
import sys
import time
from threading import Thread
import importlib.util

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

#reconocer personas

# Define VideoStream class to handle streaming of video from webcam in separate processing thread
# Source - Adrian Rosebrock, PyImageSearch: https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/
class VideoStream:
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self,resolution=(640,480),framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3,resolution[0])
        ret = self.stream.set(4,resolution[1])
            
        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

    # Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
    # Start the thread that reads frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
    # Return the most recent frame
        return self.frame

    def stop(self):
    # Indicate that the camera and thread should be stopped
        self.stopped = True

# Define and parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--modeldir', help='Folder the .tflite file is located in',
                    required=True)
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                    default='detect.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                    default='labelmap.txt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                    default=0.5)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.',
                    default='1280x720')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
                    action='store_true')

args = parser.parse_args()

MODEL_NAME = args.modeldir
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu

# Import TensorFlow libraries
# If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
# If using Coral Edge TPU, import the load_delegate library
pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
    if use_TPU:
        from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if use_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

# If using Edge TPU, assign filename for Edge TPU model
if use_TPU:
    # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
    if (GRAPH_NAME == 'detect.tflite'):
        GRAPH_NAME = 'edgetpu.tflite'       

# Get path to current working directory
CWD_PATH = os.getcwd()

# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Have to do a weird fix for label map if using the COCO "starter model" from
# https://www.tensorflow.org/lite/models/object_detection/overview
# First label is '???', which has to be removed.
if labels[0] == '???':
    del(labels[0])

# Load the Tensorflow Lite model.
# If using Edge TPU, use special load_delegate argument
if use_TPU:
    interpreter = Interpreter(model_path=PATH_TO_CKPT,
                            experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
    print(PATH_TO_CKPT)
else:
    interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

# Check output layer name to determine if this model was created with TF2 or TF1,
# because outputs are ordered differently for TF2 and TF1 models
outname = output_details[0]['name']

if ('StatefulPartitionedCall' in outname): # This is a TF2 model
    boxes_idx, classes_idx, scores_idx = 1, 3, 0
else: # This is a TF1 model
    boxes_idx, classes_idx, scores_idx = 0, 1, 2

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

# Initialize video stream
videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
time.sleep(1)

def camara(frame_rate_calc):
    #for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
    while True:

        # Start timer (for calculating frame rate)
        t1 = cv2.getTickCount()

        # Grab frame from video stream
        frame1 = videostream.read()

        # Acquire frame and resize to expected shape [1xHxWx3]
        frame = frame1.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()

        # Retrieve detection results
        boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
        scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects

        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * imH)))
                xmin = int(max(1,(boxes[i][1] * imW)))
                ymax = int(min(imH,(boxes[i][2] * imH)))
                xmax = int(min(imW,(boxes[i][3] * imW)))
                
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                # Draw label
                object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                
                if (scores[i] >= 0.50):
                    if(object_name == 'person'):
                        #print("xmax = ", xmax)
                        #print("xmin = ", xmin)
                        #print("ymax = ", ymax)
                        #print("ymin = ", ymin)
                        x = int((xmin+((xmax-xmin)/2)))
                        y = int((ymin+((ymax-ymin)/2)))
                        cv2.circle(frame,(x,y),5,(255,0,0),2)
                        found_match = 1
                        print("person")

        # Draw framerate in corner of frame
        cv2.putText(frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

        # All the results have been drawn on the frame, so it's time to display it.
        cv2.imshow('Object detector', frame)

        # Calculate framerate
        t2 = cv2.getTickCount()
        time1 = (t2-t1)/freq
        frame_rate_calc= 1/time1

        # Press 'q' to quit
        #cv2.destroyAllWindows()
        #videostream.stop()
        
        try:
            return x,y,frame_rate_calc
        except UnboundLocalError:
            return -1,-1,frame_rate_calc

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
    #print('leyendo magnetometro...')
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
    #print(v)
    f.ChangeDutyCycle(v)
#calcular el angulo de rotacion del robot
def angulo(target, pos1,x):
    #ang debe ser la orientacion del robot leida por los nodos del magnetometro
    ang = leer_compas()

    ang = (ang*mt.pi)/180

    ubicacion = leer_gps(pos1)
    #print(ubicacion)
    if x == 0:
        posicion_x = ubicacion[0]
        posicion_y = ubicacion[1] 
        x=x+1

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

    #regular la velocidad del motor segun la posicion
    d = mt.sqrt(((target[0]-ubicacion[0])**2)+((target[1]-ubicacion[1])**2))
    print("distancia = ", d*10000)
    #convertir la distancia en un valor entre 50 y 80 para manejar el motor
    d = map(d*10000,0,5,50,80)
    #un filtro para no pasarnos de 50 o de 80
    d = min(max(50,d),80)

    kit.servo[14].angle = d
    print("vel = ", d)
    return ang_gi, ubicacion,x
#mover el servo
def servo(ang):

    x = ang + 90
    x = min(max(0,x),180)
    if x > 160:
        x = 160
    elif x < 20:
        x = 20
    else:
        pass
    
    kit.servo[13].angle = x
#mapear los datos 
def map(x, in_min, in_max, out_min, out_max):
		mapped =  float((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)
		return mapped  

#inicio del programa
x = float(input("cual es la longitud?:  "))
y = float(input("cual es la latitud?:  "))

x1 = 0
target = np.array([x,y])

while True:
    ang, pos, x1 = angulo(target,pos,x1)

    servo(ang)

    x,y,frame_rate_calc = camara(frame_rate_calc)
    print("(",x,",",y,")")
    
    print(ang)

