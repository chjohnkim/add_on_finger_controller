#!/usr/bin/env python
import sys
import math
import rospy
from rospy import init_node, is_shutdown
from dynamixel_msgs.msg import JointState
from std_msgs.msg import *

from dynamixel_controllers.srv import *
from ur10_motion_script.msg import regrasp


##___Global Variables___###
goal_pos = float;
goal_speed = 1.0;
pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
psi_current = None
length = None


###___Call set_speed service to set speed of motor (Range: 0~255)___###
def increment_speed(increment):
    global goal_speed

    rospy.wait_for_service('/tilt_controller/set_speed')
    try:
        setspeed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed) 
        if increment == 1:
            setspeed(goal_speed + 0.2)
            goal_speed = goal_speed + 0.2
            print (goal_speed)
        if increment == -1:
            setspeed(goal_speed - 0.2)
            goal_speed = goal_speed - 0.2
            print (goal_speed)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

###___Set speed of the linear actuator from a range of 0 to 2___###
def set_speed(speed):

    rospy.wait_for_service('/tilt_controller/set_speed')
    try:
        setspeed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed) 
        setspeed(speed)
        print (speed)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e  


###___Set position of the linear actuator where 0 is fully contracted and 255 is fully extended___###
def set_position(position):
    global goal_pos
    
    extend_max = -2;
    contract_max = 1.5;
    goal_pos = position
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))
    goal_pos = 1.5-(goal_pos/73.0)

    if goal_pos < extend_max:
        goal_pos = extend_max
    if goal_pos > contract_max:
        goal_pos = contract_max
    pub.publish(Float64(goal_pos))
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))


###___Get command from user to control the gripper (e > extend, c > contract, f > faster, s > slower, 0~255 position)___###
def get_Key():
    user_command = raw_input()   
    if user_command in ['e']:
        set_position(255)
        print("Extend")
    elif user_command in ['c']:
        set_position(0)
        print("Contract")
    elif user_command in ['f']:
        increment_speed(1)
        print("Faster")
    elif user_command in ['s']:
        increment_speed(-1)
        print("Slower")
    elif user_command in ['x']:
        set_speed(0)
        print("Emergency Stop")
    else:
        set_pos = int(user_command)
        if set_pos <= 255 and user_command >= 0:
            set_position(set_pos)  
  
###___Callback Function___###
def callback(data):
    get_Key()

def feedforward(regrasp_status):
    print regrasp_status.data
    if regrasp_status.data in ['regrasping']:
        print 'looping!'
        rospy.Subscriber('/psi_current', Float32, get_psi)
        rospy.Subscriber('/length_value', Float32, get_length)
        a = length * math.cos(math.radians(psi_current))
        gripper = 237 #length from parallel gripper base to 2-finger tip excluding wire mount
        tip = 27.5 #length from tip of third finger to distal link far joint
        base = 60 #length from parallel gripper base to third finger proximal link base joint
        L = gripper - tip - base - a
        theta = (L - 154.655)/(-0.732758) #Linearized equation at theta = 20degrees 
        position = -41*theta/18 + 225 #Relation between proximal link angle and servo input value 
        position = round(position, 0)
        set_position(position)
        print L, theta
        #calculate the input value to the servo motor
    if regrasp_status.data in ['pushing']:
        set_position(180)
        rospy.sleep(2)
        set_position(165)
        rospy.sleep(2)
        set_position(230)
        rospy.sleep(2)        
        set_position(165)
        rospy.sleep(2)
        set_position(250)
        rospy.sleep(2)
        set_position(165) 
        
 
def get_psi(data):
    global psi_current
    psi_current = data.data


def get_length(data):
    global length
    length = data.data
    
     
###___Initiate node; subscribe to topic; call callback function___###
def dynamixel_control():
    print 'hello0'
    rospy.init_node('dynamixel_control', anonymous=True)
    #rospy.Subscriber('/tilt_controller/state', JointState, callback)
    rospy.Subscriber('/regrasp_status', String, feedforward)
    rospy.spin()

###___MAIN___###
if __name__ == '__main__':

    try:
        dynamixel_control()
    except rospy.ROSInterruptException: pass
