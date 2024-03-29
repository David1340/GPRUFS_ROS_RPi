#!/usr/bin/env python3

#Bibliotecas do ros
import rospy as rp
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#Outras bibliotecas
from LidarX2 import LidarX2
import numpy as np
import time
import serial 
import cv2

#Modelo cinemático do robô
R = 5.3 #raio das rodas em cm
d = 11.25/2 #distância entre as rodas e o centro do robô em cm
modelo_cinematico = 0.5*np.array([[R,R],[R/d,- R/d]])
modelo_cinematico_inverso = np.linalg.inv(modelo_cinematico)
Linear_maximo = 5.3
Angular_maximo = 0.9422

#camera
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
camera.set(cv2.CAP_PROP_FPS,30)
image = Image()
bridge = CvBridge()

#LidarX2
lidarx2 = LidarX2("/dev/ttyUSB0")  # Name of the serial port, can be /dev/tty*, COM*, etc.
lidar_msg = Float32MultiArray()
if not lidarx2.open():
    print("Cannot open lidarX2")
    exit(1)

#Comunicação USB com o stm32
stm32 = serial.Serial(port = '/dev/ttyACM0',baudrate=115200)
vel = Int16MultiArray()

#Inicia o nó robot 
rp.init_node('robot', anonymous= False)
# anynomous: mata outro node com nome igual na inicializacao
# caso contrario, adiciona um numero na frente do novo node

def convert_to_scalar(measures):
    angles = []
    distances = []
    for measure in measures:
        angle,distance = measure.get_pair()
        angles.append(angle*(np.pi/180))
        distances.append(distance)
    return angles + distances

def enviar_velocidade(msg):
    global  modelo_cinematico_inverso, Linear_maximo, Angular_maximo, stm32

    V = np.array([[msg.linear.x,msg.angular.z]]).T #[v;W]
 
    if(np.abs(V[0]) > np.abs(Linear_maximo)):
        V[0] = Linear_maximo * V[0]/np.abs(V[0])

    if(np.abs(V[1]) > np.abs(Angular_maximo)):
        V[1] = Angular_maximo * V[1]/np.abs(V[1])

    FI = modelo_cinematico_inverso @ V #[fi_d;fi_e]

    msg2 = [255,0,0]
    if(FI[0] >= 0):  
        msg2[1] = round(100*np.abs(FI[0][0]))
    else:
        msg2[1] = -round(100*np.abs(FI[0][0])) + 127
    msg2[1] = int(msg2[1])
    if(FI[1] >= 0):
        msg2[2] = round(100*np.abs(FI[1][0]))
    else:
        msg2[2] = -round(100*np.abs(FI[1][0])) + 127
    msg2[2] = int(msg2[2])
    stm32.write(msg2)

#funções de callBack
def callBack_cmd_vel(msg):
    global vel,image,lidar_msg

    pub_lidarx2.publish(lidar_msg)
    pub_vel.publish(vel)  
    pub_camera.publish(image)
    
    enviar_velocidade(msg)
    
sub = rp.Subscriber('/robot/cmd_vel', Twist, callBack_cmd_vel)
pub_vel = rp.Publisher('/robot/encoder',Int16MultiArray,queue_size=1)
pub_lidarx2 = rp.Publisher('/robot/lidar',Float32MultiArray,queue_size=1)
pub_camera = rp.Publisher('/robot/camera',Image,queue_size=1)

try:

    while(not rp.is_shutdown()):
        #leitura dos encoders
        if(stm32.inWaiting() > 1):
            msg_read = stm32.readline(3).decode().split()
            vel.data = [int(aux) for aux in msg_read]

        #leitura da câmera
        ret, frame = camera.read()
        if(ret):
            image = bridge.cv2_to_imgmsg(frame,'bgr8')
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            image = bridge.cv2_to_imgmsg(frame_gray,'mono8')
            
        # Get latest lidar measures
        measures = lidarx2.getMeasures()  
        if(len(measures) > 0): 
            lidar_msg.data = convert_to_scalar(measures)

except KeyboardInterrupt:
    stm32.close()
    lidarx2.close()
    print("Execução encerrada pelo usuário")