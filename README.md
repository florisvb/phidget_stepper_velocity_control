# Example ROS node for controlling a Phidgets stepper

Phidgets links:
* https://www.phidgets.com/docs21/1067_User_Guide
* https://www.phidgets.com/?view=api

# Installation

* Clone this directory into your catkin workspace: `git clone xxxx`
* Run `$ catkin_make` from the catkin base directory

# Running the node

* With `roscore` running, launch the node with instructions to the topic to listen to (Float32 topic) 
    ```rosrun phidget_stepper_velocity_control stepper_velocity_control.py --topic=test```
* Test it with:
    ```rostopic pub test std_msgs/Float32 1000```

# How this package was made

* `$ catkin_create_pkg phidget_stepper_velocity_control std_msgs rospy roscpp`
* Created python file in directory `nodes`
* Made python node executable: `$ chmod +x stepper_velocity_control.py`