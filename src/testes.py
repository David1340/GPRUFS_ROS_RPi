#!/usr/bin/env python3

#Biblioteca Raspberryi Pi
import RPi.GPIO as gpio
from rpi_hardware_pwm import HardwarePWM
from encoder import Encoder

#Outras bibliotecas
import numpy as np
import time

#Pinagem
gpio.setmode(gpio.BCM)
sentido_direito = 26
gpio.setup(sentido_direito,gpio.OUT)

#Configurando PWM 
freq = 50*10**3
motor_direito = HardwarePWM(pwm_channel=0, hz=freq) #GPIO_12
DutyCycle = 0
motor_direito.start(DutyCycle)
estado_direito = False
gpio.output(sentido_direito,estado_direito)
#encoder
#e1 = Encoder(19, 13)

while(True):
    print(DutyCycle, "\n")
    motor_direito.change_duty_cycle(100 - DutyCycle)
    time.sleep(2)
    #print("Value is {} \n".format(e1.getValue()))
    DutyCycle = DutyCycle + 10
    if(DutyCycle > 100): 
        DutyCycle = 0
        estado_direito = not estado_direito
        gpio.output(sentido_direito,estado_direito)
    
