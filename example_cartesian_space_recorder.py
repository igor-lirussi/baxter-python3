#!/usr/bin/env python3
import time
import argparse
import rospy
import baxter
import cv2
import numpy as np
import csv  
from baxter_interface import Gripper
from baxter_interface import CHECK_VERSION


parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type=str, default='left', help='Arm, left or right')
parser.add_argument('-r', '--record_rate', type=int, default='100', help='milliseconds after which another point is recorded, remember the update of robot is 100 Hz, so no lower than 10ms or values will be duplicated')
parser.add_argument('-f', '--file', type=str, default='data_record', help='the file name to record to')
args = parser.parse_args()

SIDE = args.arm
RECORDRATE=args.record_rate
FILENAME=args.file


LAST_TIME=int(time.time_ns() / 1_000_000)
WIDTH = 960
HEIGHT = 600
img = np.zeros((int(HEIGHT), int(WIDTH), 3), dtype=np.uint8) #black base image

print('Initialize robot')
rospy.init_node("Recorder")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(rate=100, arm=SIDE)
rospy.sleep(2.0)
robot.set_robot_state(True)

print('Initialize gripper')
gripper = Gripper(SIDE, CHECK_VERSION)
if gripper.error():
    gripper.reset()
if (not gripper.calibrated() and
    gripper.type() != 'custom'):
    gripper.calibrate()

print('Get robot pose')
p = robot._endpoint_state.pose.position
print('->Current Position    x:%.2f y:%.2f z:%.2f'%(p.x,p.y,p.z))
q = robot._endpoint_state.pose.orientation
print('->Current Orientation x:%.2f y:%.2f z:%.2f w:%.2f'%(q.x,q.y,q.z,q.w))

run=True
record=False
data = []
print("\nPress Wheel Button on Arm to start recording")
print("Press Back Button on Arm to stop recording")
print("Press Show Button on Arm (lower one than wheel) to exit.\n You can record now!\n")
while not rospy.is_shutdown() and run:
    # Look for gripper button presses
    if robot._hand_lower_button_state.state:
        gripper.open()
    if robot._hand_upper_button_state.state:
        gripper.close()
    #pass between recording state and not recording
    if record:
        img = np.zeros((int(HEIGHT), int(WIDTH), 3), dtype=np.uint8) #black base image
        cv2.putText(img, "RECORDING", (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (0,255,0), 3)
        cv2.putText(img, f"({len(data)} points, {int(time.time_ns() / 1_000_000)-time_start_record}ms)", (300,500), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (0,255,0), 3)
        NOW=int(time.time_ns() / 1_000_000)
        if (NOW-LAST_TIME)>=RECORDRATE:
            LAST_TIME=NOW
            #add position and gripper data
            p = robot._endpoint_state.pose.position
            q = robot._endpoint_state.pose.orientation            
            data.append([NOW-time_start_record, p.x, p.y, p.z, q.x, q.y, q.z, q.w, robot._gripper_state.position, robot._gripper_state.force])  # Append data to the list
            print('.', end='')
        #check button pressed CANCEL_BUTTON
        if robot._navigator_state.buttons[1] == True:
            print(f"({len(data)} points, {int(time.time_ns() / 1_000_000)-time_start_record}ms)")
            print("STOP RECORDING")
            record=False
            #save file
            fullfilename = f"{FILENAME}_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv"
            with open(fullfilename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "________p_x________", "________p_y________", "________p_z________", "________q_x________", "________q_y________", "________q_z________", "________q_w________", "gripper_position", "gripper_force"])  # Write header
                writer.writerows(data)  # Write data to the CSV file
            data = []  # Clear the data list
    else:
        img = np.zeros((int(HEIGHT), int(WIDTH), 3), dtype=np.uint8) #black base image
        cv2.putText(img, "NOT recording", (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (0,0,255), 3)
        #check button pressed OK_BUTTON (the wheel)
        if robot._navigator_state.buttons[0] == True:
            print("START RECORDING", end='')
            record=True
            time_start_record=int(time.time_ns() / 1_000_000)
            data = []

    #check button pressed SHOW_BUTTON
    if robot._navigator_state.buttons[2] == True:
        print("Exiting")
        run=False
        img = np.zeros((int(HEIGHT), int(WIDTH), 3), dtype=np.uint8) #black base image
        cv2.putText(img, "Exit", (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255,0,0), 3)

    robot._set_display_data(cv2.resize(img, (1024,600)))

    