"""
Description: Interface to code Baxter Robot in Python, import it in your project as described in the README!
             Each object "BaxterRobot" created with this class represents a robotic hand of Baxer, you can create two.
Author: Igor Lirussi (https://igor-lirussi.github.io)
"""
#!/usr/bin/env python3
from copy import deepcopy

import rospy
import cv_bridge
from baxter_core_msgs.msg import JointCommand, EndpointState, CameraSettings, CameraControl
from baxter_core_msgs.msg import EndEffectorCommand, EndEffectorProperties, EndEffectorState, EndpointState, NavigatorState, DigitalIOState
from baxter_core_msgs.srv import OpenCamera, CloseCamera, SolvePositionIK, SolvePositionIKRequest
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import JointState, Image
from sensor_msgs.msg import Range
import json


class BaxterRobot:

    def __init__(self, arm, rate=100):
        self.rate = rospy.Rate(rate)

        self.name = arm
        self._cartesian_pose = {}
        self._cartesian_velocity = {}
        self._cartesian_effort = {}
        self._joint_names = ["_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"]
        self._joint_names = [arm+x for x in self._joint_names]
        iksvc_ns = "/ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(iksvc_ns, SolvePositionIK)
        rospy.wait_for_service(iksvc_ns)
        print("IK service loaded.")

        #robot
        self._pub_robot_state = rospy.Publisher("/robot/set_super_enable", Bool, queue_size=10)
        #display
        self._pub_display = rospy.Publisher("/robot/xdisplay", Image, latch=True, queue_size=1)

        #navigators buttons on arm
        self._sub_navigator_state = rospy.Subscriber("/robot/navigators/"+arm+"_navigator/state", NavigatorState, self._fill_navigator_state) 
        self._navigator_state = NavigatorState()

        #joints
        self._pub_joint_cmd = rospy.Publisher("/robot/limb/"+arm+"/joint_command", JointCommand, queue_size=1)
        self._sub_joint_state = rospy.Subscriber("/robot/joint_states", JointState, self._fill_joint_state)
        self._joint_angle = {}
        self._joint_velocity = {}
        self._joint_effort = {}

        #end effector (hand) state
        self._sub_endpoint_state = rospy.Subscriber("/robot/limb/"+arm+"/endpoint_state", EndpointState, self._fill_endpoint_state)
        self._endpoint_state = EndpointState()

        #hand upper/lower button
        self._sub_hand_upper_button_state = rospy.Subscriber("/robot/digital_io/"+arm+"_upper_button/state", DigitalIOState, self._fill_hand_upper_button_state)
        self._hand_upper_button_state = DigitalIOState()
        self._sub_hand_lower_button_state = rospy.Subscriber("/robot/digital_io/"+arm+"_lower_button/state", DigitalIOState, self._fill_hand_lower_button_state)
        self._hand_lower_button_state = DigitalIOState()

        #camera hand
        self._sub_cam_image = rospy.Subscriber("/cameras/"+arm+"_hand_camera/image", Image, self._fill_cam_image)
        self._cam_image = Image()
        
        #range sonar hand
        self._sub_ir_range = rospy.Subscriber("/robot/range/"+arm+"_hand_range/state", Range, self._fill_ir_range)
        self._ir_range = Range()

        #gripper hand
        self._pub_gripper = rospy.Publisher("/robot/end_effector/"+arm+"_gripper/command", EndEffectorCommand, queue_size=10)

        self._sub_gripper_state = rospy.Subscriber("/robot/end_effector/"+arm+"_gripper/state", EndEffectorState, self._fill_gripper_state)
        self._gripper_state = EndEffectorState()


    def _fill_navigator_state(self, msg):
        self._navigator_state = msg

    def joint_angle(self):
        return deepcopy(self._joint_angle)

    def set_robot_state(self, state):
        msg = Bool()
        msg.data = state
        self._pub_robot_state.publish(msg)

    def set_cartesian_position(self, position, orientation, override_current_movement=False):
        """
        THIS METHOD IS BLOCKING TILL THE ROBOT REACHES THE POSITION, use override_current_movement=True for the NON BLOCKING
        """
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")
        msg = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=position[0],
                    y=position[1],
                    z=position[2]
                ),
                orientation=Quaternion(
                    x=orientation[0],
                    y=orientation[1],
                    z=orientation[2],
                    w=orientation[3]
                )
            )
        )
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(msg)
        resp = self.iksvc(ikreq)
        if resp.isValid[0]:
            positions_payload={
                self.name+"_s0": resp.joints[0].position[0],
                self.name+"_s1": resp.joints[0].position[1],
                self.name+"_e0": resp.joints[0].position[2],
                self.name+"_e1": resp.joints[0].position[3],
                self.name+"_w0": resp.joints[0].position[4],
                self.name+"_w1": resp.joints[0].position[5],
                self.name+"_w2": resp.joints[0].position[6],
            }
            if override_current_movement:
                self.set_joint_position(positions_payload)
            else:
                self.move_to_joint_position(positions_payload)
        else:
            print("[Error] position invalid! "+str(resp))
        return resp.isValid[0]

    def set_joint_position(self, positions):
        self._command_msg = JointCommand()
        self._command_msg.names = list(positions.keys())
        self._command_msg.command = list(positions.values())
        self._command_msg.mode = JointCommand.POSITION_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_velocity(self, velocities):
        self._command_msg = JointCommand()
        self._command_msg.names = list(velocities.keys())
        self._command_msg.command = list(velocities.values())
        self._command_msg.mode = JointCommand.VELOCITY_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def move_to_joint_position(self, positions, timeout=15.0):
        """
        THIS METHOD IS BLOCKING, TILL THE ROBOT REACHES THE POSITION, use set_joint_position for the NON BLOCKING
        """
        current_angle = self.joint_angle()
        end_time = rospy.get_time() + timeout
        
        # update the target based on the current location
        # if you use this instead of positions, the jerk will be smaller.
        def current_target():
            for joint in positions:
                current_angle[joint] = 0.012488 * positions[joint] + 0.98751 * current_angle[joint]
            return current_angle

        def difference():
            diffs = []
            for joint in positions:
                diffs.append(abs(positions[joint] - self._joint_angle[joint]))
            return diffs
        # sleep till position is reached
        while any(diff > 0.008726646 for diff in difference()) and rospy.get_time() < end_time:
            self.set_joint_position(current_target())
            self.rate.sleep()
        return all(diff < 0.008726646 for diff in difference())
    
    def move_to_neutral(self):
        angles = dict(list(zip(self._joint_names, [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0])))
        return self.move_to_joint_position(angles)

    def move_to_zero(self):
        angles = dict(list(zip(self._joint_names, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])))
        return self.move_to_joint_position(angles)

    def _fill_joint_state(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_angle[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]

    def _fill_endpoint_state(self, msg):
        self._endpoint_state = msg

    def _fill_hand_upper_button_state(self, msg):
        self._hand_upper_button_state = msg

    def _fill_hand_lower_button_state(self, msg):
        self._hand_lower_button_state = msg

    def _fill_cam_image(self, msg):
        self._cam_image = msg

    def _fill_ir_range(self, msg):
        self._ir_range = msg

    def _fill_gripper_state(self, msg):
        self._gripper_state = msg

    def _set_display_data(self, image):
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
        self._pub_display.publish(msg)

    def _set_camera(self, camera_name, state, width=640, height=400, fps=30):
        if state:
            rospy.wait_for_service("/cameras/open")
            camera_proxy = rospy.ServiceProxy("/cameras/open", OpenCamera)
            settings = CameraSettings()
            settings.width = width
            settings.height = height
            settings.fps = fps
            #gain
            control = CameraControl.CAMERA_CONTROL_GAIN #gain 0 to 79 or -1 for auto
            value = -1
            lookup = [c for c in settings.controls if c.id == control]
            try:
                lookup[0].value = value
            except IndexError:
                settings.controls.append(CameraControl(control, value))
            #exposure
            control = CameraControl.CAMERA_CONTROL_EXPOSURE #exposure 0 to 100 or -1 for auto
            value = -1
            lookup = [c for c in settings.controls if c.id == control]
            try:
                lookup[0].value = value
            except IndexError:
                settings.controls.append(CameraControl(control, value))
            #get response
            response = camera_proxy(camera_name, settings)
            return response
        else:
            rospy.wait_for_service("/cameras/close")
            camera_proxy = rospy.ServiceProxy("/cameras/close", CloseCamera)
            response = camera_proxy(camera_name)
            return response


    #GRIPPER  
    def gripper_set(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_SET
        self._pub_gripper.publish(_command_end_effector)

    def gripper_configure(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_CONFIGURE
        self._pub_gripper.publish(_command_end_effector)

    def gripper_reboot(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_REBOOT
        self._pub_gripper.publish(_command_end_effector)

    def gripper_reset(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_RESET
        self._pub_gripper.publish(_command_end_effector)

    def gripper_calibrate(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_CALIBRATE
        self._pub_gripper.publish(_command_end_effector)

    def gripper_clear_calibration(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_CLEAR_CALIBRATION
        self._pub_gripper.publish(_command_end_effector)

    def gripper_prepare_to_grip(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_PREPARE_TO_GRIP 
        self._pub_gripper.publish(_command_end_effector)

    def gripper_grip(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_GRIP
        self._pub_gripper.publish(_command_end_effector)

    def gripper_release(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_RELEASE
        self._pub_gripper.publish(_command_end_effector)

    def gripper_go(self, position=0):
        if position<0:
            position=0
        if position>100:
            position=100
        arguments = json.dumps({"position":position})
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_GO
        _command_end_effector.args = arguments
        self._pub_gripper.publish(_command_end_effector)

    def gripper_stop(self):
        _command_end_effector = EndEffectorCommand()
        _command_end_effector.id =  self._gripper_state.id
        _command_end_effector.command = EndEffectorCommand.CMD_STOP
        self._pub_gripper.publish(_command_end_effector)

