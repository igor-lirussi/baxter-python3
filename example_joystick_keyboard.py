#!/usr/bin/env python3
import argparse
import rospy
import numpy as np
from pynput.keyboard import Key, Listener
import baxter #here we are importing the baxter.py interface. (cause it's in this same folder, but in your project please clone the repo as submodule and import the interface as described in the readme)

global desired_position
global desired_orientation

parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type=str, default='left', help='Arm: left or right (default: left)')
args = parser.parse_args()

SIDE = args.arm
print("Arm "+SIDE+" chosen, change it with -a option")

def on_press(key):
    print()
    p = robot._endpoint_state.pose.position
    q = robot._endpoint_state.pose.orientation
    # I need to update the desired position with the ROBOT_position+delta (so I do not go too far away with the desired position)
    # but only on the direction selected or I accumulate error on the other directions!
    delta = 0.03
    #position
    if key.char=="w":
        desired_position[0]=p.x+delta
    
    if key.char=="s":
        desired_position[0]=p.x-delta
        
    if key.char=="a":
        desired_position[1]=p.y+delta

    if key.char=="d":
        desired_position[1]=p.y-delta        

    if key.char=="k":
        desired_position[2]=p.z+delta  

    if key.char=="l":
        desired_position[2]=p.z-delta
        
    #orientation
    if key.char=="r":
        desired_orientation[0]=q.x+delta
    
    if key.char=="f":
        desired_orientation[0]=q.x-delta
    
    if key.char=="t":
        desired_orientation[1]=q.y+delta
    
    if key.char=="g":
        desired_orientation[1]=q.y-delta
    
    if key.char=="y":
        desired_orientation[2]=q.z+delta
    
    if key.char=="h":
        desired_orientation[2]=q.z-delta
    
    if key.char=="u":
        desired_orientation[3]=q.w+delta
    
    if key.char=="j":
        desired_orientation[3]=q.w-delta
    
    #gripper
    if key.char=="m":
        robot.gripper_grip()

    if key.char=="n":
        robot.gripper_release()
    
    
    #print('Current Position    x:%.2f y:%.2f z:%.2f'%(p.x,p.y,p.z))
    #print('Desired Position    x:%.2f y:%.2f z:%.2f'%(desired_position[0],desired_position[1],desired_position[2]))
    print('Position Current x:%.2f y:%.2f z:%.2f Desired: x:%.2f y:%.2f z:%.2f'%(p.x,p.y,p.z,desired_position[0],desired_position[1],desired_position[2]))
    #print('Current Orientation x:%.2f y:%.2f z:%.2f w:%.2f'%(q.x,q.y,q.z,q.w))
    #print('Desired Orientation x:%.2f y:%.2f z:%.2f w:%.2f'%(desired_orientation[0],desired_orientation[1],desired_orientation[2],desired_orientation[3]))
    #print('Orientation Current x:%.2f y:%.2f z:%.2f w:%.2f Desired: x:%.2f y:%.2f z:%.2f w:%.2f'%(q.x,q.y,q.z,q.w,desired_orientation[0],desired_orientation[1],desired_orientation[2],desired_orientation[3]))
    
    #SEND THE ROBOT TO DESIRED POSITION
    robot.set_cartesian_position( np.array(desired_position) , np.array(desired_orientation), override_current_movement=True )


def on_release(key):
    if key.char == ('q'):
        return False





print('Initialize robot')
rospy.init_node("Joystick")
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
print('Update desired pose')
desired_position=[0,0,0]
desired_orientation=[0,0,0,0]
desired_position[0]=p.x
desired_position[1]=p.y
desired_position[2]=p.z
desired_orientation[0]=q.x
desired_orientation[1]=q.y
desired_orientation[2]=q.z
desired_orientation[3]=q.w
print('done!')

delta_position=[0.0,0.0,0.0]
delta_orientation=[0.0,0.0,0.0,0.0]

with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

