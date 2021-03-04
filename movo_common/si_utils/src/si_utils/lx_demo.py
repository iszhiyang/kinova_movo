#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_python import MoveGroupInterface


from movo_action_clients.gripper_action_client import GripperActionClient


class ScrewBoltNut(object):

	def __init__(self):




	def pick(self, obj)
	'''
	obj = bolt or nut
	'''
    if obj == "nut":
    	pass

    elif obj == "bolt":
    	pass

    else:
    	rospy.logerr("! Object name not recoginzed !")


    def align(self, obj):
    	pass


    def screw(self):
    	pass


 if __name__ == "__main__":
 	action = ScrewBoltNut()

 	try:
 		action.pick("nut")
 		rospy.sleep(2.0)

 		action.pick("bolt")
 		rospy.sleep(2.0)

 		action.align("nut")
 		rospy.sleep(2.0)

 		action.align("bolt")
 		rospy.sleep(2.0)