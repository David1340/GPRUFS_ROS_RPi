#!/usr/bin/env python3

#Bibliotecas ros
import rospy as rp
from geometry_msgs.msg import Twist
#Biblioteca Raspberryi Pi
import RPi.GPIO as gpio
from rpi_hardware_pwm import HardwarePWM
#Outras bibliotecas
import numpy as np
import time

#Modelo cinemático do robô
R = 5.3 #raio das rodas em cm
d = 11.25/2 #distância entre as rodas e o centro do robô em cm
modelo_cinematico = 0.5*np.array([[R,R],[R/d,- R/d]])
modelo_cinematico_inverso = np.linalg.inv(modelo_cinematico)
Linear_maximo = 5.3
Angular_maximo = 0.9422

#Pinagem
#gpio.setmode(gpio.BOARD) #se você quer se referir aos pinos pela sua numeração
gpio.setmode(gpio.BCM) #se você quer se referir aos pinos da mesma forma que o fabricante
sentido_direito = 26
sentido_esquerdo = 19
gpio.setup(sentido_direito,gpio.OUT)
gpio.setup(sentido_esquerdo,gpio.OUT)

#Configurando pwms
freq = 50*10**3 #Hz
motor_direito = HardwarePWM(pwm_channel=0, hz=freq) #GPIO_12
motor_esquerdo = HardwarePWM(pwm_channel=1, hz=freq) #GPIO_13
motor_direito.start(0)
motor_esquerdo.start(0)

#Inicia o nó robot 
rp.init_node('robot', anonymous= False)
# anynomous: mata outro node com nome igual na inicializacao
# caso contrario, adiciona um numero na frente do novo node

#funções de callBack
def callBack_cmd_vel(msg):
    global motor_direito, motor_esquerdo, sentido_direito, sentido_esquerdo,  modelo_cinematico_inverso, Linear_maximo, Angular_maximo
    V = np.array([[msg.linear.x,msg.angular.z]]).T #[v;W]

    FI = modelo_cinematico_inverso @ V #[fi_d;fi_e]

    if(np.abs(V[0]) > np.abs(Linear_maximo)):
        V[0] = Linear_maximo * V[0]/np.abs(V[0])

    if(np.abs(V[1]) > np.abs(Angular_maximo)):
        V[1] = Angular_maximo * V[1]/np.abs(V[1])

    if(FI[0] >= 0):  
        gpio.output(sentido_direito,False)
    else:
        gpio.output(sentido_direito,True)
    motor_direito.change_duty_cycle(100 - 100*np.abs(FI[0]))

    if(FI[1] >= 0):
        gpio.output(sentido_esquerdo,False)
    else:
        gpio.output(sentido_esquerdo,True)
    motor_esquerdo.change_duty_cycle(100 - 100*np.abs(FI[1]))

    print('pwm direito:',100 - 100*np.abs(FI[0]),'\n')
    print('pwm esquerdo:',100 - 100*np.abs(FI[1]),'\n')

sub = rp.Subscriber('/robot/cmd_vel', Twist, callBack_cmd_vel)

# evita a finalizacao do script enquanto espera pelas mensagens
rp.spin()