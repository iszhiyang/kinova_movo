#!/usr/bin/env python  
import rospy
import math
import tf

import geometry_msgs.msg
from tf.transformations import *
from geometry_msgs.msg import PoseStamped,TransformStamped, Point, Quaternion, Vector3

class my_transformer(object):
	def __init__(self):
		self.tf = tf.TransformerROS(True, rospy.Duration(10.0))
		# self.listener = tf.TransformListener()

		t1 = TransformStamped()
		# /alvar /nut_start
		t1.header.frame_id = '/ar_marker_17'
		t1.child_frame_id = '/nutStart'
		# t1.transform.translation = Vector3(-0.057, 0.050, -0.012)
		# t1.transform.translation = Vector3(-0.052, 0.164, -0.009)
		t1.transform.translation = Vector3(-0.052, 0.194, -0.009)
		qua_t1 = self.r_to_q([0, 0, -math.pi/2], [0, 0, 0, 1])
		t1.transform.rotation = Quaternion(qua_t1[0], qua_t1[1], qua_t1[2], qua_t1[3])

		'''
		t2 = TransformStamped()
		# /camera /alvar
		(trans, rot) = self.listener.lookupTransform('/movo_camera_color_optical_frame', '/ar_marker_17', rospy.Time(0))
		t2.header.frame_id = '/movo_camera_color_optical_frame'
		t2.child_frame_id = '/ar_marker_17'
		t2.transform.translation = Vector3(trans[0], trans[1], trans[2])
		t2.transform.rotation = Quaternion(rot[0], rot[1], rot[2], rot[3])
		'''
		
		t3 = TransformStamped()
		# /left_ee /finger
		t3.header.frame_id = '/left_ee_link'
		t3.child_frame_id = '/finger'
		t3.transform.translation.x = 0.043
		t3.transform.translation.z = 0.02
		t3.transform.rotation = Quaternion(0, 0, 0, 1)
		  


		self.tf.setTransform(t1)
		# self.tfROS.setTransform(t2)
		self.tf.setTransform(t3)


	def r_to_q(self, r_list, q_org):
		rx, ry, rz = r_list
		q_rot = quaternion_from_euler(rx, ry, rz)
		q_new = quaternion_multiply(q_rot, q_org)
		return q_new

	def point_comb(self, point1, point2, p_frame):
		# point = (trans, rot)
		# point = result from lookupTransform
		res = PoseStamped()
		res.header.frame_id = p_frame
		(tran1, rot1) = point1
		(tran2, rot2) = point2
		res.pose.position.x = tran1[0] + tran2[0]
		res.pose.position.y = tran1[1] + tran2[1]
		res.pose.position.z = tran1[2] + tran2[2]
		res_q = quaternion_multiply(rot1, rot2)
		res.pose.orientation = Quaternion(res_q[0], res_q[1], res_q[2], res_q[3])
		return res
