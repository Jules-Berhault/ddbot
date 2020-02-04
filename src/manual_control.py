#!/usr/bin/env python
# license removed for brevity
import curses
import rospy
from std_msgs.msg import Float64

global cmd_left
global cmd_right

cmd_left=0
cmd_right=0
def goback():
    global cmd_left
    global cmd_right
    pwm=min(cmd_left,cmd_right)
    cmd_left=max(min(pwm-10,255),0)
    cmd_right=max(min(pwm-10,255),0)

def turn_left():
    global cmd_left
    global cmd_right
    cmd_left=max(min(cmd_left-1,255),0)
    cmd_right=max(min(cmd_right+1,255),0)

def turn_right():
    global cmd_left
    global cmd_right
    cmd_left=max(min(cmd_left+1,255),0)
    cmd_right=max(min(cmd_right-1,255),0)

def goline():
    global cmd_left
    global cmd_right
    pwm=max(cmd_left,cmd_right)
    cmd_left=max(min(pwm+1,255),0)
    cmd_right=max(min(pwm+1,255),0)


stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)


stdscr.refresh()

key = ''

rospy.init_node('manual_control', anonymous=True)
pub_u1 = rospy.Publisher('u1', Float64, queue_size=10)
pub_u2 = rospy.Publisher('u2', Float64, queue_size=10)
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    """ on diminue la commande """
    

    key = stdscr.getch()
    
    if key == curses.KEY_UP: 
        goline()
    elif key == curses.KEY_LEFT: 
        turn_left()
    elif key == curses.KEY_RIGHT: 
        turn_right()
    elif key == curses.KEY_DOWN: 
        goback()
    elif key == 's': 
        cmd_left=0
        cmd_right=0

    pub_u1.publish(cmd_left)
    pub_u2.publish(cmd_right)
    print(cmd_left,cmd_right)
    rate.sleep()


curses.endwin()

