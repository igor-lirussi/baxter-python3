"""
Description: Simple example of how to move the robotic arm of baxter
Author: Igor Lirussi (https://igor-lirussi.github.io)
"""
#!/usr/bin/env python3
import rospy
import baxter #here we are importing the baxter.py interface. (cause it's in this same folder, but in your project please clone the repo as submodule and import the interface as described in the readme)
rospy.init_node("example")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(arm="left")
rospy.sleep(2.0)
robot.set_robot_state(True)
robot.move_to_neutral()
robot.move_to_zero()
robot.move_to_joint_position({"left_s0": 1.0})
robot.move_to_joint_position({"left_s0": -1.0})
robot.move_to_neutral()
robot.set_robot_state(False)

