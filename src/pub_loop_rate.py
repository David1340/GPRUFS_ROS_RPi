#!/usr/bin/env python3

import rospy as rp
from geometry_msgs.msg import Twist

rp.init_node('node')

msg = Twist()
msg.linear.x = 1
msg.angular.z = 0.2

pub = rp.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)


rate = rp.Rate(10) # 10hz

while not rp.is_shutdown():
   pub.publish(msg)
   rate.sleep()