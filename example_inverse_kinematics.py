import time
import rospy
import baxter
import cv2
import numpy as np

rospy.init_node("testing")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(rate=100, arm="left")
rospy.sleep(2.0)
# robot._set_camera(camera_name="left_hand_camera", state=True, width=WIDTH, height=HEIGHT, fps=30)

robot.set_robot_state(True)
#get position of hand
p = robot._endpoint_state.pose.position
q = robot._endpoint_state.pose.orientation
print("Position:")
print(p)
print("Orientation:")
print(q)

#make the robot move in square with inverse kinematics to 4 different points 
delta = 0.1
robot.set_cartesian_position([p.x-delta, p.y-delta, p.z], [q.x, q.y, q.z, q.w])
robot.set_cartesian_position([p.x+delta, p.y-delta, p.z], [q.x, q.y, q.z, q.w])
robot.set_cartesian_position([p.x+delta, p.y+delta, p.z], [q.x, q.y, q.z, q.w])
robot.set_cartesian_position([p.x-delta, p.y+delta, p.z], [q.x, q.y, q.z, q.w])
robot.set_cartesian_position([p.x-delta, p.y-delta, p.z], [q.x, q.y, q.z, q.w])


# while not rospy.is_shutdown():
#     robot.rate.sleep()

# print(robot.move_to_neutral())
# robot.set_robot_state(False)
