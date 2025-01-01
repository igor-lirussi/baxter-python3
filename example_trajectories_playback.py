"""
Description: Replay the trajectories recorded of the robot arm(s). Works best with trajectories from the recorder.
             Be careful of your columns order/total number and call the right option, the code will assume the positions of the data to play (cartesian data, or joint data) depending on the number of columns available and on the option specified. 
             The standard recording formats and columns order are:
             (one hand recording)           | 1st Time | 2nd - 3rd Rostime(unused) | 4th -> 10th cartesian position | 11th - 12th gipper pos and force | 
             (one hand + joint recording)   | 1st Time | 2nd - 3rd Rostime(unused) | 4th -> 10th cartesian position | 11th - 12th gipper pos and force | 13th -> 19th joint position |
             (both hands recording)         | 1st Time | 2nd - 3rd Rostime(unused) | 4th -> 10th cartesian position | 11th - 12th gipper pos and force | 13th -> 19th cartesian dx position | 20th - 21th dx gipper pos and force |
             (both hands + joints recording)| 1st Time | 2nd - 3rd Rostime(unused) | 4th -> 10th cartesian position | 11th - 12th gipper pos and force | 13th -> 19th joint dx position | 20th -> 26th cartesian dx position | 27th - 28th gipper dx pos and force | 29th -> 35th joint dx position |
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
parser.add_argument('-a', '--arm', type=str, default='left', help='Arm: left or right (default: left) or both')
parser.add_argument('-f', '--file', type=str, help='the path to the file name to play, it has to be a .csv (1st COLUMN: time from 0 in ms, COLUMNS 4th to 10th: endpoint position and orientation, 11th COLUMN: gripper position)')
parser.add_argument('-r', '--playback_interval', type=int, default='-1', help='milliseconds (default: -1, inferred from data) after which another point is played (es 100ms=10Hz), remember the update of robot is 100 Hz, so no lower than 10ms or values may be duplicated')
parser.add_argument('-gf', '--grip_on_force', action='store_true', help='Instead of commanding the gripper to a specific position, command the gripper to fully close only when gripper force is positive in the data (NEEDS 12th COLUMN) (add this argument to activate)')
parser.add_argument('-j', '--joints', action='store_true', help='Plays from the 7 joints positions (NEEDS COLUMNS 13th to 19th) (joint space) instead of the cartesian space (add this argument to activate)')
args = parser.parse_args()

SIDE = args.arm
FILENAME=args.file
PLAYBACKRATE=args.playback_interval

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

print("-> "+SIDE+" arm chosen, change it with -a option")
both_arms=False
if SIDE=="both":
    both_arms=True
    SIDE="left" #to create the first as normal
    print("ATTENTION: BOTH arms selected")
    print("Be careful of your columns order/total number and call the right option, the code will assume the positions of the data to play (cartesian data, or joint data) depending on the number of columns available and on the option specified.")
    if len(data[0])<21:
        print("FILE INVALID for playback with two arms, not enough columns, check --help")
        exit()
    if args.joints and len(data[0])<35:
        print("FILE INVALID for playback with two arms with JOINTS, not enough columns, check --help")
        exit()


if PLAYBACKRATE==-1:
    print("Inferring playback interval....")
    PLAYBACKRATE=(data[-1,0]-data[0,0])/len(data)
print('Playback Interval: {}'.format(PLAYBACKRATE))

print('Initialize robot')
rospy.init_node("Playback")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(rate=100, arm=SIDE)
if both_arms:
    robot_r = baxter.BaxterRobot(rate=100, arm="right")
rospy.sleep(2.0)
robot.set_robot_state(True)

print('Initialize gripper')
robot.gripper_calibrate()
rospy.sleep(4.0)
string = "Calibrated: {} Ready: {} Moving: {} Gripping: {}".format(robot._gripper_state.calibrated, robot._gripper_state.ready, robot._gripper_state.moving, robot._gripper_state.gripping)
print(string)
if both_arms:
    print('Initialize gripper right arm')
    robot_r.gripper_calibrate()
    rospy.sleep(4.0)
    string = "Calibrated: {} Ready: {} Moving: {} Gripping: {}".format(robot_r._gripper_state.calibrated, robot_r._gripper_state.ready, robot_r._gripper_state.moving, robot_r._gripper_state.gripping)
    print(string)

print('Get robot pose:')
p = robot._endpoint_state.pose.position
print('Current Position    x:%.2f y:%.2f z:%.2f'%(p.x,p.y,p.z))
q = robot._endpoint_state.pose.orientation
print('Current Orientation x:%.2f y:%.2f z:%.2f w:%.2f'%(q.x,q.y,q.z,q.w))
if both_arms:
    print('Get robot right arm pose:')
    p = robot_r._endpoint_state.pose.position
    print('Current Position    x:%.2f y:%.2f z:%.2f'%(p.x,p.y,p.z))
    q = robot_r._endpoint_state.pose.orientation
    print('Current Orientation x:%.2f y:%.2f z:%.2f w:%.2f'%(q.x,q.y,q.z,q.w))

WIDTH = 960
HEIGHT = 600
img = np.zeros((int(HEIGHT), int(WIDTH), 3), dtype=np.uint8) #black base image
cv2.putText(img, "POSITIONING", (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255,0,0), 3)
robot._set_display_data(cv2.resize(img, (1024,600)))

print("\nMOVING TO FIRST POSITION!")
if args.joints:
    # only here we use move_to_joint_position because is blocking the control flow, later we will use set_joint_position
    robot.move_to_joint_position({SIDE+"_s0": data[0, 12], SIDE+"_s1": data[0, 13], SIDE+"_e0": data[0, 14], SIDE+"_e1": data[0, 15], SIDE+"_w0": data[0, 16], SIDE+"_w1": data[0, 17], SIDE+"_w2": data[0, 18]})
    if both_arms:
        robot_r.move_to_joint_position({"right_s0": data[0, 28], "right_s1": data[0, 29], "right_e0": data[0, 30], "right_e1": data[0, 31], "right_w0": data[0, 32], "right_w1": data[0, 33], "right_w2": data[0, 34]})
else: #using cartesian
    px = data[0, 3]
    py = data[0, 4]
    pz = data[0, 5]
    qx = data[0, 6]
    qy = data[0, 7]
    qz = data[0, 8]
    qw = data[0, 9]
    # only here we use set_cartesian_position because is blocking the control flow, later we will use it non blocking
    movement_valid = robot.set_cartesian_position([px, py, pz], [qx, qy, qz, qw])
    if both_arms:
        if len(data[0])>=35: # we suppose the file has also both joints for the hands, so other arm cartesian position columns are 20th to 26th
            robot_r.set_cartesian_position([data[0, 19], data[0, 20], data[0, 21]], [data[0, 22], data[0, 23], data[0, 24], data[0, 25]])
        else: # we suppose the file has only cartesian columns for the hands, so other arm cartesian position columns are 13th to 19th
            robot_r.set_cartesian_position([data[0, 12], data[0, 13], data[0, 14]], [data[0, 15], data[0, 16], data[0, 17], data[0, 18]])
    if movement_valid:
        print("[info] Movement OK")
if args.grip_on_force:
    gripper_force = data[0, 11]
    if gripper_force > 0.1:
        robot.gripper_grip()
    else:
        robot.gripper_release()
    if both_arms:
        if len(data[0])>=35: # we suppose the file has also both joints for the hands, so other arm gripper force is 28th
            gripper_force = data[0, 27]
        else:# we suppose the file has only cartesian columns for the hands, so other arm gripper force is 21st
            gripper_force = data[0, 20]
        if gripper_force > 0.1:
            robot_r.gripper_grip()
        else:
            robot_r.gripper_release()
else: # grip normally
    gripper_position = data[0, 10]
    robot.gripper_go(gripper_position)
    if both_arms:
        if len(data[0])>=35: # we suppose the file has also both joints for the hands, so other arm gripper position is 27th
            gripper_position = data[0, 26]
            robot_r.gripper_go(gripper_position)
        else:# we suppose the file has only cartesian columns for the hands, so other arm gripper position is 20st
            gripper_position = data[0, 19]
            robot_r.gripper_go(gripper_position)

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
            robot.set_joint_position({SIDE+"_s0": data[index, 12], SIDE+"_s1": data[index, 13], SIDE+"_e0": data[index, 14], SIDE+"_e1": data[index, 15], SIDE+"_w0": data[index, 16], SIDE+"_w1": data[index, 17], SIDE+"_w2": data[index, 18]}) #NON BLOCKING
            if both_arms:
                robot_r.set_joint_position({"right_s0": data[0, 28], "right_s1": data[0, 29], "right_e0": data[0, 30], "right_e1": data[0, 31], "right_w0": data[0, 32], "right_w1": data[0, 33], "right_w2": data[0, 34]}) #NON BLOCKING
        else: #using cartesian
            px = data[index, 3]
            py = data[index, 4]
            pz = data[index, 5]
            qx = data[index, 6]
            qy = data[index, 7]
            qz = data[index, 8]
            qw = data[index, 9]
            movement_valid = robot.set_cartesian_position([px, py, pz], [qx, qy, qz, qw], override_current_movement=True) #NON BLOCKING
            if both_arms:
                if len(data[0])>=35: # we suppose the file has also both joints for the hands, so other arm cartesian position columns are 20th to 26th
                    robot_r.set_cartesian_position([data[0, 19], data[0, 20], data[0, 21]], [data[0, 22], data[0, 23], data[0, 24], data[0, 25]], override_current_movement=True) #NON BLOCKING
                else: # we suppose the file has only cartesian columns for the hands, so other arm cartesian position columns are 13th to 19th
                    robot_r.set_cartesian_position([data[0, 12], data[0, 13], data[0, 14]], [data[0, 15], data[0, 16], data[0, 17], data[0, 18]], override_current_movement=True) #NON BLOCKING
            if not movement_valid:
                print("[error] Movement FAILED")
        if args.grip_on_force:
            gripper_force = data[index, 11]
            if gripper_force > 0.1:
                robot.gripper_grip()
            else:
                robot.gripper_release()
            if both_arms:
                if len(data[0])>=35: # we suppose the file has also both joints for the hands, so other arm gripper force is 28th
                    gripper_force = data[0, 27]
                else:# we suppose the file has only cartesian columns for the hands, so other arm gripper force is 21st
                    gripper_force = data[0, 20]
                if gripper_force > 0.1:
                    robot_r.gripper_grip()
                else:
                    robot_r.gripper_release()
        else: # grip normally
            gripper_position = data[index, 10]
            robot.gripper_go(gripper_position)
            if both_arms:
                if len(data[0])>=35: # we suppose the file has also both joints for the hands, so other arm gripper position is 27th
                    gripper_position = data[0, 26]
                    robot_r.gripper_go(gripper_position)
                else:# we suppose the file has only cartesian columns for the hands, so other arm gripper position is 20st
                    gripper_position = data[0, 19]
                    robot_r.gripper_go(gripper_position)
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
    