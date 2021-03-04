#!/usr/bin/env python  
import rospy
import math
import tf
from tf.transformations import *
from geometry_msgs.msg import PoseStamped,TransformStamped, Quaternion
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from movo_action_clients.gripper_action_client import GripperActionClient
from movo_action_clients.move_base_action_client import MoveBaseActionClient
from movo_action_clients.torso_action_client import TorsoActionClient
from si_utils.lx_transformerROS import my_transformer

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

class screw_1(object):
    def __init__(self):
        # self.scene = PlanningSceneInterface("base_link")
        self.dof = "7dof"
        self.robot = moveit_commander.RobotCommander()
        self._listener = tf.TransformListener()

        self.move_group = MoveGroupInterface("upper_body", "base_link")
        self.lmove_group = MoveGroupInterface("left_arm", "base_link")
        self.rmove_group = MoveGroupInterface("right_arm", "base_link")
        self.move_group.setPlannerId("RRTConnectkConfigDefault")
        self.lmove_group.setPlannerId("RRTConnectkConfigDefault")
        self.rmove_group.setPlannerId("RRTConnectkConfigDefault")

        if "7dof" == self.dof:

            self._upper_body_joints = ["right_shoulder_pan_joint",
                                        "right_shoulder_lift_joint",
                                        "right_arm_half_joint",
                                        "right_elbow_joint",
                                        "right_wrist_spherical_1_joint",
                                        "right_wrist_spherical_2_joint",
                                        "right_wrist_3_joint",
                                        "left_shoulder_pan_joint",
                                        "left_shoulder_lift_joint",
                                        "left_arm_half_joint",
                                        "left_elbow_joint",
                                        "left_wrist_spherical_1_joint",
                                        "left_wrist_spherical_2_joint",
                                        "left_wrist_3_joint",
                                        "linear_joint",
                                        "pan_joint",
                                        "tilt_joint"]
            self._right_arm_joints = ["right_shoulder_pan_joint",
                                        "right_shoulder_lift_joint",
                                        "right_arm_half_joint",
                                        "right_elbow_joint",
                                        "right_wrist_spherical_1_joint",
                                        "right_wrist_spherical_2_joint",
                                        "right_wrist_3_joint"]
            self._left_arm_joints = ["left_shoulder_pan_joint",
                                        "left_shoulder_lift_joint",
                                        "left_arm_half_joint",
                                        "left_elbow_joint",
                                        "left_wrist_spherical_1_joint",
                                        "left_wrist_spherical_2_joint",
                                        "left_wrist_3_joint"]
            self.tucked = [-1.6,-1.5,0.4,-2.7,0.0,0.5, -1.7,1.6,1.5,-0.4,2.7,0.0,-0.5,1.7, 0.04, 0, 0]
            # self.constrained_stow =[-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, 1.0, 2.6, -2.0, 0.0, -2.0, 0.0, 0.0, -1.0, 0.42, 0, 0]
            self.constrained_stow = [-2.037, 1.046, -2.877, -1.423, -1.143, 1.679, 1.690, 2.037, -1.046, 2.877, 1.423, 1.143, -1.679, -1.690, 0.4, 0.288, 0]
        else:
            rospy.logerr("DoF needs to be set at 7, aborting demo")
            return


        self._lgripper = GripperActionClient('left')
        self._rgripper = GripperActionClient('right')
        # self.scene.clear()

    def goto_tuck(self):
        # remove previous objects
        raw_input("========== Press Enter to goto_tuck ========")
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(self._upper_body_joints, self.tucked, 0.05, max_velocity_scaling_factor=0.4)
            
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def goto_plan_grasp(self):
        raw_input("========== Press Enter to goto_plan_grasp ========")
        while not rospy.is_shutdown():

            try:
                result = self.move_group.moveToJointPosition(self._upper_body_joints, self.constrained_stow, 0.05, max_velocity_scaling_factor=0.3)
            
            except:
                continue

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

     
    def insert_bolt(self):
        # right arm
        # base_link y+
        pose = self.curr_pose('right_ee_link')
        pose.pose.position.y += 0.05
        raw_input("=========== Press Enter to insert bolt ========")
        # rospy.sleep(2.0)
        while not rospy.is_shutdown():

            try:
                result = self.rmove_group.moveToPose(pose, "right_ee_link", tolerance=0.02, max_velocity_scaling_factor=0.1)
            
            except:
                continue

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
        

    def insert_nut(self):
        pass


    def start(self):
        pass


    def retreat(self):
        pass


    def curr_pose(self, eef):
        pose = PoseStamped()
        pose.header.frame_id = eef
        curr_pose = self._listener.transformPose('/base_link', pose)
        return curr_pose



if __name__=='__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("screw_1_demo")

    g = screw_1()
    g.insert_bolt()


    # try:
        # g.goto_tuck()
        # g.goto_plan_grasp()

        # g.insert_bolt()

    # except rospy.ROSInterruptException:
        # pass
