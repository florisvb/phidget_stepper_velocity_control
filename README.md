# Example ROS node for controlling a Phidgets stepper

Phidgets links:
* https://www.phidgets.com/docs21/1067_User_Guide
* https://www.phidgets.com/?view=api

# Installation

### Install the ROS package
* Clone this directory into your catkin workspace: `$ git clone https://github.com/florisvb/phidget_stepper_velocity_control.git`
* Run `$ catkin_make` from the catkin base directory

### Create a udev rule
* Check your phidget serial number: plug it in, and type `$ dmesg`, note the serial number
* Edit the file `99-phidget-stepper-505962.rules` to reflect the correct serial number
* Copy the updated rules file to `/etc/udev/rules.d` (ie. `sudo cp 99-phidget-stepper-505962.rules /etc/udev/rules.d`)
* Reload your rules: `$ sudo udevadm control --reload-rules && sudo udevadm trigger`

# Running the node

* With `roscore` running, launch the node with instructions to the topic to listen to (Float32 topic):
* `$ rosrun phidget_stepper_velocity_control stepper_velocity_control.py --topic=test`
* Test it with:
* `$ rostopic pub test std_msgs/Float32 1000`

# How this package was made

* `$ catkin_create_pkg phidget_stepper_velocity_control std_msgs rospy roscpp`
* Created python file in directory `nodes`
* Made python node executable: `$ chmod +x stepper_velocity_control.py`