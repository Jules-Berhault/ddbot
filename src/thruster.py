#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from arduino_driver import *
""" Variable du Noeud """
global cmd_u1
global cmd_u2
global arduino 
global cmd_max
cmd_max=255
cmd_u1=40
cmd_u2=0



def callback_u1(data):
    global cmd_u1
    cmd_u1=data.data
     

def callback_u2(data):
    global cmd_u2
    cmd_u2=data.data

""" initialisation du node """
rospy.init_node('thruster_node', anonymous=True)

""" creation subscriber """
sub_u1 = rospy.Subscriber('u1', Float64, callback_u1)
sub_u2 = rospy.Subscriber('u2', Float64, callback_u2)

""" initialisation arduino """
arduino=init_arduino_line()

rate = rospy.Rate(25) # 25hz


while not rospy.is_shutdown():

    send_arduino_cmd_motor(arduino,clip_cmd(cmd_u1,cmd_max),clip_cmd(cmd_u2,cmd_max))
    
    rate.sleep()

