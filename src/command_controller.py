#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

""" Variable du Noeud """

global cmd_u1
global cmd_u2
global arduino 
global cmd_max
global spdmax # ticks/s
global u1_speed# ticks/s
global u2_speed
global dt
global dOdoL
global dOdoR
global errL 
global errR
errL = 0
errR = 0
dOdoL=0
dOdoR=0
dt=0.1
cmd_max=255
cmd_u1=0
cmd_u2=0
spdmax=300
u1_speed = 0 # ticks/s
u2_speed=0

def callback_u1(data):
    global u1_speed
    global spdmax
    u1_speed=spdmax*(data.data/255)
     

def callback_u2(data):
    global u2_speed
    global spdmax
    u2_speed=spdmax*(data.data/255)

def callback_delta_odo_1(data):
    global dOdoL
    dOdoL=data.data

def callback_delta_odo_2(data):
    global dOdoR
    dOdoR=abs(data.data)

""" initialisation du node """
rospy.init_node('command_controller_node', anonymous=True)

""" creation subscriber sortie controller """
sub_u1 = rospy.Subscriber('u1', Float64, callback_u1)
sub_u2 = rospy.Subscriber('u2', Float64, callback_u2)
sub_delta_odo1=rospy.Subscriber('delta_odo_1', Float64, callback_delta_odo_1)
sub_delta_odo2=rospy.Subscriber('delta_odo_2', Float64, callback_delta_odo_2)

""" creation publisher  """
pub_cmd_u1 = rospy.Publisher('cmd_u1', Float64,queue_size=1000)
pub_cmd_u2 = rospy.Publisher('cmd_u2', Float64,queue_size=1000)

rate = rospy.Rate(1/dt) # 10hz

while not rospy.is_shutdown():
    
    """ regulation de moteur par encodeur """
    
    if u1_speed==0 or u2_speed==0:
        if u1_speed==0:
            cmd_u1=0
        if u2_speed==0:
            cmd_u2=0    
    else:
        errL = u1_speed-dOdoL
        errR = u2_speed-dOdoR
        kp = 0.3
        cmd_u1 = cmd_u1 + kp*errL
        cmd_u2 = cmd_u2 + kp*errR
    if cmd_u1>255:
        cmd_u1=255
    if cmd_u2>255:
        cmd_u2=255
    if cmd_u1<0:
        cmd_u1=0
    if cmd_u2<0:
        cmd_u2=0

    """ publication des commande moteur """

    pub_cmd_u1.publish(cmd_u1)
    pub_cmd_u2.publish(cmd_u2)

    print(spdmax,u1_speed,u2_speed,errL,errR,dOdoL,dOdoR,cmd_u1,cmd_u2)

    rate.sleep()
