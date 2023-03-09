#!/usr/bin/env python3

#Bibliotecas ros
import rospy as rp
from geometry_msgs.msg import Twist
#Biblioteca Raspberryi Pi
import RPi.GPIO as gpio
#Outras bibliotecas
import numpy as np
import time

#Modelo cinemático do robô
R = 5.3 #raio das rodas em cm
d = 11.25/2 #distância entre as rodas e o centro do robô em cm
modelo_cinematico = 0.5*np.array([[R,R],[R/d,- R/d]])
modelo_cinematico_inverso = np.linalg.inv(modelo_cinematico)

#Pinagem
gpio.setmode(gpio.BOARD) #se você quer se referir aos pinos pela sua numeração
motor_direito = 37
sentido_direito = 35
motor_esquerdo = 38
sentido_esquerdo = 36

#Configurando os pinos como saída
gpio.setup(motor_direito,gpio.OUT) 
gpio.setup(sentido_direito,gpio.OUT) 
gpio.setup(motor_esquerdo,gpio.OUT) 
gpio.setup(sentido_esquerdo,gpio.OUT) 

#Configurando pwms
freq = 10*10**3 #10k Hz
pwm_direito = gpio.PWM(motor_direito,freq)  #pino e frequência
pwm_esquerdo = gpio.PWM(motor_esquerdo,freq)  #pino e frequência
pwm_direito.start(0) #inicia o pwm com duty cicle igual a 0.
pwm_esquerdo.start(0) #inicia o pwm com duty cicle igual a 0.

#Inicia o nó robot 
rp.init_node('robot', anonymous= False)
# anynomous: mata outro node com nome igual na inicializacao
# caso contrario, adiciona um numero na frente do novo node

#funções de callBack
def callBack_cmd_vel(msg):
    global pwm_direito, pwm_esquerdo, sentido_direito, sentido_esquerdo, modelo_cinematico_inverso
    V = np.array([[msg.linear.x,msg.angular.z]]).T #[v;W]
    FI = modelo_cinematico_inverso @ V #[fi_d;fi_e]
    if(FI[0] >= 0):
        sentido_direito = True
    else:
        sentido_direito = False

    if(FI[1] >= 0):
        sentido_esquerdo = True
    else:
        sentido_esquerdo = False

    print('velocidade.linear.x: ',msg.linear.x,'\n')
    print('velocidade.angular.z: ',msg.angular.z,'\n')

sub = rp.Subscriber('/robot/cmd_vel', Twist, callBack_cmd_vel)

# evita a finalizacao do script enquanto espera pelas mensagens
rp.spin()