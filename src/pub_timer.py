#!/usr/bin/env python3

import rospy as rp
from std_msgs.msg import String
from geometry_msgs.msg import Twist
rp.init_node('node1', anonymous=True)
# anynomous: mata outro node com nome igual na inicializacao
# caso contrario, adiciona um numero na frente do novo node

msg = Twist()
msg.linear.x = 1
msg.angular.z = 0.2

pub = rp.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)


def timerCallBack(event):
    pub.publish(msg)

# timer com 0.1s de periodo (10hz)
timer = rp.Timer(rp.Duration(0.1), timerCallBack)


# evita a finalizacao do script
rp.spin()
