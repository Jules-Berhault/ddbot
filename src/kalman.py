from roblib import *
import geometry_msgs.msg._PoseWithCovarianceStamped

def observateur(X,Xb, Gx, dt):
    Xf = X.flatten()
    x, y, theta, v = Xf
    u1 = v*cos(theta)
    u2 = v*sin(theta)
    u = dt*array([[u1, u2]]).T
    A = eye(2)
    C = eye(2) 
    Galpha = zeros((2,2))
    Gbeta = 100*eye(2)
     
    Y = array([[x, y]]).T
    Xb, Gx = kalman(Xb, Gx, u, Y, Galpha, Gbeta, A, C)
    return Xb, Gx

def callback_u1:


""" initialisation du node """
rospy.init_node('observer_node', anonymous=True)

""" creation subscriber """
state_publisher = rospy.Subscriber('state', PoseWithCovarianceStamped, callback_u1)
sub_u2 = rospy.Subscriber('u2', Float64, callback_u2)

X = array([[5, 5, 0.1, 0.]]).T
u_c = array([[0.8, 0.9]]).T
dt = 1

Xb = array([[0.1, 0.1]]).T
Gx = 1000*eye(2)

rate = rospy.Rate(1) # 25hz

while not rospy.is_shutdown():
    Xb, Gx = observateur(X,Xb, Gx, dt)