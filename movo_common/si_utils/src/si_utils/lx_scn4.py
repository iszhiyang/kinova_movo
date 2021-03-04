#!/usr/bin/env python

import copy
import actionlib
import rospy
import sys
import tf
import moveit_commander
import math

from si_utils.lx_pick_place import GraspingClient, PointHeadClient

from moveit_msgs.msg import Constraints, OrientationConstraint

from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from movo_action_clients.gripper_action_client import GripperActionClient
from movo_action_clients.move_base_action_client import MoveBaseActionClient
from movo_action_clients.torso_action_client import TorsoActionClient

from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasp_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from shape_msgs.msg import SolidPrimitive as sp
from geometry_msgs.msg import Pose2D,PoseStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from std_msgs.msg import Bool

class Kinova_screw(object):
	def __init__():
		self.gp = GraspingClient(sim=False)
		self.gp.clearScene()
