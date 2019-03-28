#!/usr/bin/env python
import math
import rospy
from rospy import init_node, is_shutdown
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

##Global Variables
extend_max = -2;
extend_min = 1.5;
goal_pos = 0;
pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

## Publish user input from teleop_keyboard to vrep robot ##
def callback(data):
    global goal_pos
    global extend_max
    global extend_min
    #rospy.loginfo(rospy.get_name() + ': Current motor angle {0}'.format(data.current_pos))
    
    #If the motor has reached its limit, publish a new command
#    if fabs (goal_pos-data.current_pos) < 0.01:
#        if goal_pos == 0:
#            goal_pos = 3.141592
#        else:
#            goal_pos = 0
#        str = "Time: {0} Moving motor to {1}" .format(rospy.get_time(), goal_pos)
#        rospy.loginfo(str)
    rospy.loginfo('input value from -2 to 1.5:') 
    goal_pos = input()
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))

    if goal_pos < extend_max:
        goal_pos = -2
    if goal_pos > extend_min:
        goal_pos = 1.5
    pub.publish(Float64(goal_pos))
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))
     
## Subscribe to teleop_keyboard to retrieve user input ##
def dynamixel_control():
    rospy.init_node('dynamixel_control', anonymous=True)
    rospy.Subscriber('/tilt_controller/state', JointState, callback)
#   pub.publish(Float64(goal_pos))
    rospy.spin()

## MAIN ##
if __name__ == '__main__':
    try:
        dynamixel_control()
    except rospy.ROSInterruptException: pass
