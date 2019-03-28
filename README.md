# Add-on Finger Controller via Dynamixel AX-12A 

This is a ROS package to control the add-on finger device using the Dynamixel Servo AX-12A.

## Hardware requirements
- Dynamixel AX-12A Servo

- DC Power Supply

- USB2Dynamixel

## Software prerequisites
- ROS kinetic 
- dynamixel_motor package <http://wiki.ros.org/dynamixel_motor> 
- Software tested on Ubuntu 16.04.3 LTS.

## How to run it
1. To initialize motor controller
```
roslaunch add_on_finger_controller controller_manager.launch
```
2. To initialize tilt controller
```
roslaunch add_on_finger_controller start_tilt_controller.launch 
```

3. To run the example python script to control the add-on finger:
```
rosrun add_on_finger_controller example.py
```

## Authors

* **John Kim** (chkimaa@connect.ust.hk)
