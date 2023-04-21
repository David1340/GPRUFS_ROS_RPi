#!/usr/bin/env python3

#Bibliotecas ros
import rospy as rp
from geometry_msgs.msg import Twist
from gprufs.msg import Velocity
from std_msgs.msg import Float32MultiArray
#Biblioteca Raspberryi Pi
import RPi.GPIO as gpio
#Outras bibliotecas
import numpy as np
import time
import serial 

#Modelo cinemático do robô
R = 5.3 #raio das rodas em cm
d = 11.25/2 #distância entre as rodas e o centro do robô em cm
modelo_cinematico = 0.5*np.array([[R,R],[R/d,- R/d]])
modelo_cinematico_inverso = np.linalg.inv(modelo_cinematico)
Linear_maximo = 5.3
Angular_maximo = 0.9422

#Comunicação USB com o stm32
stm32 = serial.Serial(port = '/dev/ttyACM0',baudrate=115200)

#Pinagem
gpio.setmode(gpio.BCM) #se você quer se referir aos pinos da mesma forma que o fabricante

#Inicia o nó robot 
rp.init_node('robot', anonymous= False)
# anynomous: mata outro node com nome igual na inicializacao
# caso contrario, adiciona um numero na frente do novo node

vel = Float32MultiArray()
vel.data = [2.43,0.13]
cont = 0

#funções de callBack
def callBack_cmd_vel(msg):
    #global motor_direito, motor_esquerdo, sentido_direito, sentido_esquerdo,  modelo_cinematico_inverso, Linear_maximo, Angular_maximo
    global cont,vel,modelo_cinematico_inverso, Linear_maximo, Angular_maximo
    
    pub.publish(vel)
    """
    V = np.array([[msg.linear.x,msg.angular.z]]).T #[v;W]

    FI = modelo_cinematico_inverso @ V #[fi_d;fi_e]

    if(np.abs(V[0]) > np.abs(Linear_maximo)):
        V[0] = Linear_maximo * V[0]/np.abs(V[0])

    if(np.abs(V[1]) > np.abs(Angular_maximo)):
        V[1] = Angular_maximo * V[1]/np.abs(V[1])

    msg = [255,0,0]
    if(FI[0] >= 0):  
        msg[1] = round(100*np.abs(FI[0][0]))
    else:
        msg[1] = -round(100*np.abs(FI[0][0])) + 127
    msg[1] = int(msg[1])
    if(FI[1] >= 0):
        msg[2] = round(100*np.abs(FI[1][0]))
    else:
        msg[2] = -round(100*np.abs(FI[1][0])) + 127
    msg[2] = int(msg[2])
    #print(msg)
    #stm32.write(msg)
    pub.publish(vel)
    cont = cont + 1
    #print(cont,'\n')
    #print('pwm direito:',100*np.abs(FI[0][0]),'\n')
    #print('pwm esquerdo:',100*np.abs(FI[1][0]),'\n')
    """

sub = rp.Subscriber('/robot/cmd_vel', Twist, callBack_cmd_vel)
pub = rp.Publisher('/robot/vel',Float32MultiArray,queue_size=1)

while(True):
    if(stm32.inWaiting() >0):
        msg_read = stm32.readline(3).decode().split()
        vel.data = [int(msg_read[0]),int(msg_read[1])]

# evita a finalizacao do script enquanto espera pelas mensagens
rp.spin()