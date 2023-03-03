#!/usr/bin/env python3

import rospy as rp
from geometry_msgs.msg import Twist

rp.init_node('robot', anonymous= False)

# anynomous: mata outro node com nome igual na inicializacao
# caso contrario, adiciona um numero na frente do novo node

# callback do topico cmd_vel
def topiccallBack(msg):
    global valor
    valor = msg.linear.x
    print('velocidade.linear.x: ',valor,'\n')

# subscriber
# arg: topic, tipo_de_msg, callback
sub = rp.Subscriber('/robot/cmd_vel', Twist, topiccallBack)

# evita a finalizacao do script enquanto espera pelas mensagens
rp.spin()