#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

import time
import encoders_driver_py3 as encodrv

def delta_odo (odo1,odo0):
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo

""" initialisation du node """
rospy.init_node('encoder_node', anonymous=True)

""" Publisher """
pub_encoder_1 = rospy.Publisher("delta_odo_1", Float64, queue_size = 10)
pub_encoder_2 = rospy.Publisher("delta_odo_2", Float64, queue_size = 10)

encodrv.set_baudrate(baudrate=115200)

sync0, timeAcq0, sensLeft0, sensRight0, posLeft0, posRight0 =  encodrv.read_single_packet(debug=True)

# Rate Loop
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    sync1, timeAcq1, sensLeft1, sensRight1, posLeft1, posRight1 =  encodrv.read_single_packet(debug=True)

    dOdoL = abs(delta_odo(posLeft1,posLeft0))
    dOdoR = abs(delta_odo(posRight1,posRight0))

    pub_encoder_1.publish(dOdoL)
    pub_encoder_2.publish(dOdoR)
    
    posRight0 = posRight1
    posLeft0 = posLeft1
    rate.sleep()