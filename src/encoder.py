#!/usr/bin/env python
<<<<<<< HEAD
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
=======
# license removed for brevity

#from roblib import *
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler
import rospy
from numpy.linalg import inv
from numpy import *
from matplotlib.pyplot import scatter, show

theta = 0 
v = 0
#X = array([[5, 5, 0.1, 0.]]).T
u_c = array([[0.8, 0.9]]).T
freq = 10
dt = 1/freq
Y = array([[0, 0]]).T
Xb = array([[0.1, 0.1]]).T
Gx = 100*eye(2)


def kalman_predict(xup,Gup,u,lambalpha,A):
    lamb1 = matmul(matmul(A, Gup), A.T) + lambalpha
    x1 = matmul(A, xup) + u    
    return(x1,lamb1)    

def kalman_correc(x0,lamb0,y,lambbeta,C):
    
    S = matmul(matmul(C, lamb0), C.T) + lambbeta        
    K = matmul(matmul(lamb0,C.T), inv(S))           
    ytilde = y - matmul(C, x0)        
    Gup = matmul((eye(len(x0))-matmul(K, C)), lamb0) 
    xup = x0 + matmul(K,ytilde)
    return(xup,Gup) 
    
def kalman(x0,lamb0,u,y,lambalpha,lambbeta,A,C):
    xup,Gup = kalman_correc(x0,lamb0,y,lambbeta,C)
    x1,lamb1=kalman_predict(xup,Gup,u,lambalpha,A)
    return(x1,lamb1)     



def observateur(theta, v, Y, Xb, Gx, dt):
    
    u1 = v*cos(theta)
    u2 = v*sin(theta)
    u = dt*array([[u1, u2]]).T
    A = eye(2)
    C = eye(2) 
    Galpha = 10*eye(2)
    Gbeta = 100*eye(2)
    Xb, Gx = kalman(Xb, Gx, u, Y, Galpha, Gbeta, A, C)
    return Xb, Gx

def capCallback(data):
    theta = data.data

def posCallback(data):
    Y[0] = data.point.x
    Y[1] = data.point.y

def  velCallback(data):
    v = sqrt(data.twist.linear.x**2 + data.twist.linear.y**2)



""" initialisation du node """
rospy.init_node('observer_node', anonymous=True)

""" creation subscriber """
state_publisher = rospy.Publisher('state', PoseStamped, queue_size=10)
cap_suscriber = rospy.Subscriber('cap', Float64, capCallback)
pos_suscriber = rospy.Subscriber('cartesian_coordinates', PointStamped, posCallback)
vel_suscriber = rospy.Subscriber('vel', TwistStamped, velCallback )

state = PoseStamped()
rate = rospy.Rate(freq) # 25hz

while not rospy.is_shutdown():
    state.header.stamp = rospy.Time.now()
    state.header.frame_id = "map"

    #scatter(Y[0], Y[1])
    
    Xb, Gx = observateur(theta, v, Y, Xb, Gx, dt)
    
    q = quaternion_from_euler(0,0, theta)
    
    state.pose.position.x = Xb[0]
    state.pose.position.y = Xb[1]

    rospy.logwarn("xb : %f,  xvrai : %f" %(Xb[0], Y[0]))
    rospy.logwarn("yb : %f,  yvrai : %f" %(Xb[1], Y[1]))
    #rospy.logwarn(" : %f" %Xb[1])

    state.pose.orientation.x = q[0]
    state.pose.orientation.x = q[1]
    state.pose.orientation.x = q[2]
    state.pose.orientation.x = q[3]

    state_publisher.publish(state)
>>>>>>> 48b5f330eb08231dc448b4ed369944aa1653f6c1
    rate.sleep()