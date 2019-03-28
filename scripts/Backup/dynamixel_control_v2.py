#!/usr/bin/env python
import sys
import math
import rospy
import termios, tty, select
from rospy import init_node, is_shutdown
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
from dynamixel_controllers.srv import *


##Global Variables
extend_max = -2;
extend_min = 1.5;
goal_pos = 0;
goal_speed = 1;
pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

## Call set_speed service to set speed of motor; limit this value to 0~2
def set_speed(speed):
    global goal_speed

    rospy.wait_for_service('/tilt_controller/set_speed')
    try:
        set_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
        if speed == 1:
            set_speed(goal_speed + 0.2)
            goal_speed = goal_speed + 0.2
            print (goal_speed)
        if speed == -1:
            set_speed(goal_speed - 0.2)
            goal_speed = goal_speed - 0.2
            print (goal_speed)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

## function to set position of the linear actuator where 0 is fully extended and 255 is fully contracted
def set_position(position):
    global extend_max
    global extend_min
    global goal_pos
 
    rospy.loginfo('input value from 0 to 255:') 
    goal_pos = position
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))
    goal_pos = (goal_pos/73.0)-2.0
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))

    if goal_pos < extend_max:
        goal_pos = -2.0
    if goal_pos > extend_min:
        goal_pos = 1.5
    pub.publish(Float64(goal_pos))
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))


## Receive user input to control the gripper
## e - extend, c - contract, f - faster, s - slower, 0~255 position
def getKey(data):
    
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    if key in ['e']:
        set_position(0)
        print("Extend")
    if key in ['c']:
        set_position(255)
        print("Contract")
    if key in ['f']:
        set_speed(1)
        print("Faster")
    if key in ['s']:
        set_speed(-1)
        print("Slower")
  
## Publish user input from teleop_keyboard to vrep robot ##
def callback(data):
    getKey()
     
## Subscribe to teleop_keyboard to retrieve user input ##
def dynamixel_control():
    rospy.init_node('dynamixel_control', anonymous=True)
    rospy.Subscriber('/tilt_controller/state', JointState, callback)
    rospy.spin()

## MAIN ##
if __name__ == '__main__':

    settings = termios.tcgetattr(sys.stdin)

    try:
        dynamixel_control()
#        set_speed(2)   
#        set_position(100)
    except rospy.ROSInterruptException: pass
