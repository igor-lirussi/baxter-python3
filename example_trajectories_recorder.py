"""
Description: Records the trajectories of the robot arm(s).
Author: Igor Lirussi (https://igor-lirussi.github.io)
"""
#!/usr/bin/env python3
import time
import argparse
import rospy
import cv2
import numpy as np
import csv  
import baxter #here we are importing the baxter.py interface. (cause it's in this same folder, but in your project please clone the repo as submodule and import the interface as described in the readme)


parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type=str, default='left', help='Arm: left or right (default: left)')
parser.add_argument('-r', '--record_rate', type=int, default='100', help='milliseconds (default: 100ms=10Hz) after which another point is recorded, remember the update of robot is 100 Hz, so no lower than 10ms or values may be duplicated')
parser.add_argument('-f', '--file', type=str, default='data_record', help='the file name to record to or path/to/filename, (default: data_record) You can also omit the extension, they are gonna be saved anyway in .csv')
parser.add_argument('-l', '--limit', type=int, default='-1', help='limit on points to record (default -1, infinite) Useful to have the same amount of points in different recordings. Stops recording automatically once recorded an amount of points')
parser.add_argument('-j', '--joints', action='store_true', help='Saves also the 7 joints positions in the last 7 column (joint space) (add this argument to activate)')
args = parser.parse_args()

SIDE = args.arm
RECORDRATE=args.record_rate
FILENAME=args.file
if FILENAME.endswith('.csv'):
    FILENAME = FILENAME[:-4]
print("Arm "+SIDE+" chosen, change it with -a option")

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
robot.gripper_calibrate()
rospy.sleep(4.0)
string = "Calibrated: {} Ready: {} Moving: {} Gripping: {}".format(robot._gripper_state.calibrated, robot._gripper_state.ready, robot._gripper_state.moving, robot._gripper_state.gripping)
print(string)

print('Get robot pose:')
p = robot._endpoint_state.pose.position
print('Current Position    x:%.2f y:%.2f z:%.2f'%(p.x,p.y,p.z))
q = robot._endpoint_state.pose.orientation
print('Current Orientation x:%.2f y:%.2f z:%.2f w:%.2f'%(q.x,q.y,q.z,q.w))

run=True
record=False
data = []
print("\n -> Press Wheel Button on Arm to start recording")
print(" -> Press Back Button on Arm to stop recording")
print(" -> Press Show Button on Arm (lower one than wheel) to exit.\n You can record now!\n")
while not rospy.is_shutdown() and run:
    # Look for gripper button presses
    if robot._hand_lower_button_state.state:
        robot.gripper_grip()
    if robot._hand_upper_button_state.state:
        robot.gripper_release()
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
            s = robot._endpoint_state.header.stamp
            row = [NOW-time_start_record, s.secs, s.nsecs, p.x, p.y, p.z, q.x, q.y, q.z, q.w, robot._gripper_state.position, robot._gripper_state.force]
            if args.joints: #if joints are recorded add in the end
                j_pos=robot._joint_angle
                row += [ j_pos[SIDE+"_s0"],j_pos[SIDE+"_s1"],j_pos[SIDE+"_e0"],j_pos[SIDE+"_e1"],j_pos[SIDE+"_w0"],j_pos[SIDE+"_w1"],j_pos[SIDE+"_w2"] ]
            data.append(row)  # Append data to the list
            print('.', end='')
        #check button pressed CANCEL_BUTTON
        if robot._navigator_state.buttons[1] == True or (args.limit>0 and len(data)>=args.limit) :
            print(f"({len(data)} points, {int(time.time_ns() / 1_000_000)-time_start_record}ms)")
            print("STOP RECORDING")
            record=False
            #save file
            fullfilename = f"{FILENAME}_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv"
            with open(fullfilename, mode='w', newline='') as file:
                writer = csv.writer(file)
                header_row=["Time","ROS secs","ROS nsecs", "________p_x________", "________p_y________", "________p_z________", "________q_x________", "________q_y________", "________q_z________", "________q_w________", "gripper_position", "gripper_force"]
                if args.joints: #if joints are recorded add in the end
                    header_row += [ SIDE+"_s0", SIDE+"_s1", SIDE+"_e0", SIDE+"_e1", SIDE+"_w0", SIDE+"_w1", SIDE+"_w2" ]
                writer.writerow(header_row)  # Write header
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
rospy.sleep(1)
    