"""
Description: Puts both hands of the robot in the neutral safe pose and deactivates the engines 
Author: Igor Lirussi (https://igor-lirussi.github.io)
"""
#!/usr/bin/env python3
import time
import rospy
import _thread
import threading
import baxter #here we are importing the baxter.py interface. (cause it's in this same folder, but in your project please clone the repo as submodule and import the interface as described in the readme)

rospy.init_node("testing")
robotL = baxter.BaxterRobot(rate=100, arm="left")
robotR = baxter.BaxterRobot(rate=100, arm="right")
rospy.sleep(2.0)

robotL.set_robot_state(True)

#sequential movements
#print(robotL.move_to_neutral())
#print(robotR.move_to_neutral())

# parallel movements of both arms
class myThread (threading.Thread):
   def __init__(self, threadID, name, robot):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.robot = robot
   def run(self):
      print ("Starting " + self.name)
      print(self.robot.move_to_neutral())
      print ("Exiting " + self.name)


thread1 = myThread(1, "Thread-1", robotL)
thread2 = myThread(2, "Thread-2", robotR)
thread1.start()
thread2.start()
thread1.join()
thread2.join()
print ("Exiting Main Thread")
robotL.set_robot_state(False)
