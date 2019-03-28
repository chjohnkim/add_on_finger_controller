#!/usr/bin/env python
import sys
import math
import rospy
from rospy import init_node, is_shutdown
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
from dynamixel_controllers.srv import *


##___Global Variables___###
goal_pos = float;
goal_speed = 1.0;
pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)


def regrasp8(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            x = PointA_world[0] - math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            z = PointA_world[2] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
             
        
        #Store Values
        pose_target.position.x = x - (object_width/2)*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))
        gposition(pub, command, position) #increment gripper width
        
        print opposite
    return [width/1000, opposite/1000]


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
     
###___Initiate node; subscribe to topic; call callback function___###
def dynamixel_control():
    rospy.init_node('dynamixel_control', anonymous=True)
    rospy.Subscriber('/tilt_controller/state', JointState, callback)
    rospy.spin()

###___MAIN___###
if __name__ == '__main__':

    try:
        dynamixel_control()
    except rospy.ROSInterruptException: pass
