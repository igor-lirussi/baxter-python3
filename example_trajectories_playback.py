"""
Description: Replay the trajectories recorded of the robot arm(s).
Author: Igor Lirussi (https://igor-lirussi.github.io)
"""
#!/usr/bin/env python3
import time
import argparse
import rospy
import cv2
import numpy as np
import csv  
import os
import baxter #here we are importing the baxter.py interface. (cause it's in this same folder, but in your project please clone the repo as submodule and import the interface as described in the readme)

parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type=str, default='left', help='Arm: left or right (default: left)')
parser.add_argument('-f', '--file', type=str, help='the path to the file name to play, it has to be a .csv (1st column: time from 0 in ms, 4th column to 10th: endpoint position and orientation, 11th column: gripper position)')
parser.add_argument('-r', '--playback_rate', type=int, default='-1', help='milliseconds (default: -1, inferred from data) after which another point is played (es 100ms=10Hz), remember the update of robot is 100 Hz, so no lower than 10ms or values may be duplicated')
parser.add_argument('-gf', '--grip_on_force', action='store_true', help='Instead of commanding the gripper to a specific position, command the gripper to fully close only when gripper force is positive in the data (NEEDS 12th column) (add this argument to activate)')
parser.add_argument('-j', '--joints', action='store_true', help='Plays from the 7 joints positions in the last 7 column (NEEDS columns 13th to 19th) (joint space) instead of the cartesian space (add this argument to activate)')
args = parser.parse_args()

SIDE = args.arm
FILENAME=args.file
PLAYBACKRATE=args.playback_rate
print("Arm "+SIDE+" chosen, change it with -a option")

print("Loading file "+FILENAME)
if os.path.isfile(FILENAME) and FILENAME.endswith('.csv'):
    # Use numpy to load
    data = np.genfromtxt(FILENAME, delimiter=',', skip_header=1)
    print("FILE Loaded!")
    if len(data[0])<11:
        print("FILE INVALID for playback, not enough columns, check --help")
        exit()
else:
    print("FILENAME INVALID, check --help")
    exit()

if args.grip_on_force:
    print("playback WITH GRIP_ON_FORCE selected")
    if len(data[0])<12:
        print("FILE INVALID for playback with GRIP_ON_FORCE, not enough columns, check --help")
        exit()

if args.joints:
    print("playback WITH JOINT POSITIONS selected")
    if len(data[0])<19:
        print("FILE INVALID for playback with JOINTS, not enough columns, check --help")
        exit()


if PLAYBACKRATE==-1:
    print("Inferring playback rate....")
    PLAYBACKRATE=(data[-1,0]-data[0,0])/len(data)
print('Playback Rate: {}'.format(PLAYBACKRATE))

print('Initialize robot')
rospy.init_node("Playback")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(rate=100, arm=SIDE)
rospy.sleep(2.0)
robot.set_robot_state(True)

print('Initialize gripper')
robot.gripper_calibrate()
rospy.sleep(4.0)
string = "Calibrated: {} Ready: {} Moving: {} Gripping: {}".format(robot._gripper_state.calibrated, robot._gripper_state.ready, robot._gripper_state.moving, robot._gripper_state.gripping)
print(string)

print('Get robot pose:')
p = robot._endpoint_state.pose.position
print('Current Position    x:%.2f y:%.2f z:%.2f'%(p.x,p.y,p.z))
q = robot._endpoint_state.pose.orientation
print('Current Orientation x:%.2f y:%.2f z:%.2f w:%.2f'%(q.x,q.y,q.z,q.w))

WIDTH = 960
HEIGHT = 600
img = np.zeros((int(HEIGHT), int(WIDTH), 3), dtype=np.uint8) #black base image
cv2.putText(img, "POSITIONING", (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255,0,0), 3)
robot._set_display_data(cv2.resize(img, (1024,600)))

print("\nMOVING TO FIRST POSITION!")
if args.joints:
    robot.move_to_joint_position({SIDE+"_s0": data[0, 12], SIDE+"_s1": data[0, 13], SIDE+"_e0": data[0, 14], SIDE+"_e1": data[0, 15], SIDE+"_w0": data[0, 16], SIDE+"_w1": data[0, 17], SIDE+"_w2": data[0, 18]})
else:
    px = data[0, 3]
    py = data[0, 4]
    pz = data[0, 5]
    qx = data[0, 6]
    qy = data[0, 7]
    qz = data[0, 8]
    qw = data[0, 9]
    movement_valid = robot.set_cartesian_position([px, py, pz], [qx, qy, qz, qw])
    if movement_valid:
        print("[info] Movement OK")
if args.grip_on_force:
    gripper_force = data[0, 11]
    if gripper_force > 0.1:
        robot.gripper_grip()
    else:
        robot.gripper_release()
else:
    gripper_position = data[0, 10]
    robot.gripper_go(gripper_position)
print("[info] Gripper OK")


rospy.sleep(1.0)
print(f"\n -> Executing trajectory of {len(data)} points\n")
run=True
LAST_TIME=int(time.time_ns() / 1_000_000)
time_start_playback=int(time.time_ns() / 1_000_000)
index=0
while not rospy.is_shutdown() and run:
    NOW=int(time.time_ns() / 1_000_000)
    #if enough time passed
    if (NOW-LAST_TIME)>=PLAYBACKRATE:
        LAST_TIME=NOW
        #print info
        img = np.zeros((int(HEIGHT), int(WIDTH), 3), dtype=np.uint8) #black base image
        string_curr_point = f"({index} point, {data[index,0]} ms, ELAPSED: {int(time.time_ns() / 1_000_000)-time_start_playback} ms)"
        cv2.putText(img, "PLAYING", (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (0,255,0), 3)
        cv2.putText(img, string_curr_point , (100,500), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (0,255,0), 3)
        robot._set_display_data(cv2.resize(img, (1024,600)))
        print(string_curr_point)
        #play the point
        if args.joints:
            robot.set_joint_position({SIDE+"_s0": data[index, 12], SIDE+"_s1": data[index, 13], SIDE+"_e0": data[index, 14], SIDE+"_e1": data[index, 15], SIDE+"_w0": data[index, 16], SIDE+"_w1": data[index, 17], SIDE+"_w2": data[index, 18]})
        else:
            px = data[index, 3]
            py = data[index, 4]
            pz = data[index, 5]
            qx = data[index, 6]
            qy = data[index, 7]
            qz = data[index, 8]
            qw = data[index, 9]
            movement_valid = robot.set_cartesian_position([px, py, pz], [qx, qy, qz, qw], override_current_movement=True)
            if not movement_valid:
                print("[error] Movement FAILED")
        if args.grip_on_force:
            gripper_force = data[index, 11]
            if gripper_force > 0.1:
                robot.gripper_grip()
            else:
                robot.gripper_release()
        else:
            gripper_position = data[index, 10]
            robot.gripper_go(gripper_position)
        #point to next index    
        index=index+1

    if index>=len(data):
        run=False
        img = np.zeros((int(HEIGHT), int(WIDTH), 3), dtype=np.uint8) #black base image
        cv2.putText(img, "FINISHED", (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255,0,0), 3)
        cv2.putText(img, f"({index} point, {int(time.time_ns() / 1_000_000)-time_start_playback}ms)", (100,500), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255,0,0), 3)
        robot._set_display_data(cv2.resize(img, (1024,600)))
        print("FINISHED")
rospy.sleep(1)
    