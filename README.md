# Baxter Python 3
**Code Baxter with Python3!**
## Description 
Useful port of baxter interface to control your Baxter with Python 3.
If you need an introductive tutorial on baxter checkout [this Baxter introduction](https://igor-lirussi.github.io/baxter-python3/BAXTER_TUTORIAL)


### Topics:
- Baxter Robot 
- Human Robot Interaction
- Python 3

## Result
![Result](./img/result.jpg)

## Requirements & Dependencies
everything should be already installed if you are running in the robot, for running it on your pc you need: 
- Python 3
- rospy
- opencv

## Install in your repository
*   add this repo in your repo: ```git submodule add git@github.com:igor-lirussi/baxter-python3.git```
*   if you want to check for updates of the submodule: ```git submodule update --remote```

*	remember to tell users to clone your repo with --recurse-submodules: ```git clone --recurse-submodules https://github.com/your_name/your_repo```
*  	if a user clones without --recurse-submodules will find the empty folder, download the submodules with ```git submodule init``` and ```git submodule update```

[For example, this is a repo that uses this module](https://github.com/igor-lirussi/Baxter-Robot-ObjDet)

## Run
```
import importlib
baxter=importlib.import_module("baxter-python3.baxter")

import rospy
rospy.init_node("example")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(arm="left")
rospy.sleep(2.0)
robot.set_robot_state(True)
#move robot
robot.move_to_neutral()
robot.move_to_zero()
robot.move_to_joint_position({"left_s0": 1.0})
robot.move_to_joint_position({"left_s0": -1.0})
robot.move_to_neutral()
#get position of hand
p = robot._endpoint_state.pose.position
q = robot._endpoint_state.pose.orientation
print(p)
#instead of the blocking
#msg = rospy.wait_for_message("/robot/limb/left/endpoint_state", EndpointState)
#p = msg.pose.position
#q = msg.pose.orientation
robot.set_robot_state(False)
```
 
check the examples for more.

### Examples
Some examples are available in the folder to demonstrate some capabilities:
- ```robotStateFalse.py``` to put both arms in normal position and deactivate the motors
- ```example_moving.py``` for moving the joints
- ```example_inverse_kinematics.py``` for moving the joints given a target point
- ```example_grippers.py``` to use the grippers
- ```example_ir_arm_distance.py``` for getting the infrared distance between grippen and object
- ```example_take_pic.py``` for getting the camera video in the hands and to take pictures with the button (useful to create a dataset of images of objects you are working on).
- ```example_cartesian_space_recorder.py``` to record trajectories in cartesian (task-space) and also joints poistion (joint space). Visualize with ```visualizer_cartesian_trajectories```

## Useful Resources & Extra
### Face expressions
This code repo, if imported in your project, allows you to give different facial expression to Baxter robot. 

Useful to warn the people around of the movements that are about to happen, looking at the place before moving the joints. 
```
import importlib
face=importlib.import_module("baxter-python3.faces")

#set looking direction
face._set_look(robot, "down")

#or display a face
face._set_face(robot, "left_down")
```

{% include_cached snippets/masonry.html internal="gallery" %}

- Links
	- https://robostack.github.io/GettingStarted.html
	- https://sdk.rethinkrobotics.com/wiki/API_Reference
	- https://sdk.rethinkrobotics.com/wiki/
	- https://sdk.rethinkrobotics.com/wiki/Foundations
	- https://sdk.rethinkrobotics.com/wiki/Advanced_Understanding
	- https://sdk.rethinkrobotics.com/wiki/Examples
	- https://sdk.rethinkrobotics.com/wiki/Customer_Videos
	- https://github.com/RethinkRobotics/baxter_common/tree/master/baxter_core_msgs/msg
	- https://github.com/RethinkRobotics/baxter_interface/tree/master/src/baxter_interface
- Resources
	- API PYTHON 2:
	  https://github.com/RethinkRobotics/baxter_interface/tree/master/src/baxter_interface

## Authors
* **Igor Lirussi** @ BOUN Boğaziçi University - CoLoRs (Cognitive Learning and Robotics) Lab
* **Alper Ahmetoglu** @ BOUN Boğaziçi University - CoLoRs (Cognitive Learning and Robotics) Lab
* **Deniz Bilge Akkoç** @ BOUN Boğaziçi University - CoLoRs (Cognitive Learning and Robotics) Lab

## Acknowledgments
*   All the people that contributed with suggestions and tips.

## License
This project is licensed - see the [LICENSE](LICENSE) file for details.
