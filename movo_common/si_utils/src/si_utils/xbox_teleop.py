#!/usr/bin/env python  
import rospy
import math
import tf
from tf.transformations import *

from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)

import sys
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose2D,PoseStamped,TransformStamped, Quaternion
from std_msgs.msg import String
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes

class xbox_teleop(object):
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self._listener = tf.TransformListener()
        self.move_group = MoveGroupInterface("upper_body", "base_link")
        self.lmove_group = MoveGroupInterface("left_arm", "base_link")
        self.rmove_group = MoveGroupInterface("right_arm", "base_link")
        # self.move_group.setPlannerId("RRTConnectkConfigDefault")
        # self.lmove_group.setPlannerId("RRTConnectkConfigDefault")
        # self.rmove_group.setPlannerId("RRTConnectkConfigDefault")

        # move_group
        # self.lmove_group = moveit_commander.MoveGroupCommander("left_arm")
        # self.lmove_group.set_goal_tolerance(0.05)
        # self.lmove_group.set_planning_time(10.0)
        # self.lmove_group.set_max_velocity_scaling_factor(0.1)

    def target(self, trans, rot):
        curr_pos = self._listener.lookupTransform('/base_link', '/left_ee_link', rospy.Time(0))
        x,y,z = trans
        r,p,y = rot
        move_rl = PoseStamped()
        move_rl.header.frame_id = "base_link"
        move_rl.pose.position.x = curr_pos[0][0] + x
        move_rl.pose.position.y = curr_pos[0][1] + y
        move_rl.pose.position.z = curr_pos[0][2] + z
        q_org = curr_pos[1]
        q_new = self.r_to_q([r, p, y], q_org)
        move_rl.pose.orientation = Quaternion(q_new[0], q_new[1], q_new[2], q_new[3])
        return move_rl

    def target_lee(self, trans, rot):
        # curr_pos = self._listener.lookupTransform('/base_link', '/left_ee_link', rospy.Time(0))
        x,y,z = trans
        r,p,y = rot
        move_rl = PoseStamped()
        move_rl.header.frame_id = "left_ee_link"
        move_rl.pose.position.x = x
        move_rl.pose.position.y = y
        move_rl.pose.position.z = z
        q_org = [0, 0, 0, 1]
        q_new = self.r_to_q([r, p, y], q_org)
        move_rl.pose.orientation = Quaternion(q_new[0], q_new[1], q_new[2], q_new[3])
        return move_rl


    def r_to_q(self, r_list, q_org):
        rx, ry, rz = r_list
        q_rot = quaternion_from_euler(rx, ry, rz)
        q_new = quaternion_multiply(q_rot, q_org)
        return q_new


if __name__ == "__main__":
    
    # Create a node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("xbox_teleop")

    g = xbox_teleop()


    # rospy.loginfo("====")

    raw_input("========= Press Enter to move ====")
    # target = g.target([0.1, 0, 0], [0, 0, 0])
    # g.lmove_group.moveToPose(target, 'left_ee_link')

    target = g.target_lee([0, 0, 0.02], [0, 0, 0])
    
    # raw_input("========= Press Enter to move ====")
    # result = g.lmove_group.moveToPose(target, 'left_ee_link')
    # g.lmove_group.set_pose_target(target, 'left_ee_link')
    # raw_input("========= Press Enter to move ====")
    # g.lmove_group.go(wait=True)
    # g.lmove_group.clear_pose_target("left_ee_link")


    while not rospy.is_shutdown():

            # raw_input("========= Press Enter to start round ====")

            try:
               
                # target = g.target_lee([0, 0, 0.02], [0, 0, 0])
                # raw_input("========= Press Enter to move ====")
                result = g.lmove_group.moveToPose(target, 'left_ee_link')
                # g.lmove_group.set_pose_target(target, 'left_ee_link')
                # rospy.sleep(1.0)             
                # g.lmove_group.go(True)
                # g.lmove_group.clear_pose_target("left_ee_link")


                # target = g.target_lee([0, 0, -0.02], [0, 0, 0])
                # raw_input("========= Press Enter to move ====")
                # result = g.lmove_group.moveToPose(target, 'left_ee_link')

                # target = g.target([0, 0, 0.1], [0, 0, 0])
                # raw_input("========= Press Enter to move ====")
                # result = g.lmove_group.moveToPose(target, 'left_ee_link')

                # g.lmove_group.set_pose_target(target, 'left_ee_link')
                # rospy.sleep(1.0)              
                # g.lmove_group.go(True)
                # g.lmove_group.clear_pose_target("left_ee_link")


            # except KeyboardInterrupt:
                # break

            except: continue

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                break