#!/usr/bin/env python  
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from si_utils.lx_transformerROS import my_transformer

if __name__ == '__main__':
    rospy.init_node('my_tf_listener')

    listener = tf.TransformListener()

    # my_trans = my_transformer()

    rate = rospy.Rate(10.0)




    while not rospy.is_shutdown():
        try:
            # look1 = listener.lookupTransform('/left_ee_link', '/link1', rospy.Time(0))
            # look2 = listener.lookupTransform('/base_link', '/left_ee_link', rospy.Time(0))

            # look3 = listener.lookupTransform('/base_link', '/link1', rospy.Time(0))

            # rospy.loginfo(look3)
            # rospy.loginfo(look2)

            pose = PoseStamped()
            pose.header.frame_id = '/link1'
            pose2 = listener.transformPose('/base_link', pose)
            rospy.loginfo(pose2)

            # (trans,rot) = listener.lookupTransform('/base_link', '/ar_marker_1', rospy.Time(0))
            # (trans,rot) = listener.lookupTransform('/base_link', '/left_ee_link', rospy.Time(0))
            # (trans1,rot1) = listener.lookupTransform('/movo_camera_color_optical_frame', '/ar_marker_17', rospy.Time(0))
            # (trans,rot) = listener.lookupTransform('/base_link', '/movo_camera_color_optical_frame', rospy.Time(0))
            # (trans,rot) = listener.lookupTransform('/movo_camera_color_optical_frame', '/base_link', rospy.Time(0))
            # (trans,rot) = listener.lookupTransform('/base_link', '/ar_marker_1', rospy.Time(0))
            # pose = PoseStamped()
            # pose.header.frame_id = 'ar_marker_1'
            # rospy.loginfo("========== First trans ===========")
            # pose1 = listener.transformPose('/movo_camera_color_optical_frame', pose)
            # rospy.loginfo(pose1)
            # rospy.loginfo("========== Second trans ===========")
            # rospy.loginfo(listener.transformPose('/base_link', pose1))




            # print(trans)
            # print(rot)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('test')



    
        


    # rate.sleep()
    
'''
    pose = PoseStamped()
    pose.header.frame_id = '/ar_marker_17'
    rospy.loginfo("========== First trans ===========")
    listener.waitForTransform("/ar_marker_17", "/movo_camera_color_optical_frame", rospy.Time(), rospy.Duration(4.0))

    pose1 = listener.transformPose('/movo_camera_color_optical_frame', pose)
    rospy.loginfo(pose1)
    rospy.loginfo("========== Second trans ===========")
    rospy.loginfo(listener.transformPose('/base_link', pose1))

    pose_nutStart_nut = PoseStamped()
    pose_nutStart_nut.header.frame_id = '/nutStart'
    pose_nutStart_ar = my_trans.tf.transformPose('/ar_marker_17', pose_nutStart_nut)
    rospy.loginfo(pose_nutStart_ar)
    pose_nutStart_ca = listener.transformPose('/movo_camera_color_optical_frame', pose_nutStart_ar)
    rospy.loginfo(pose_nutStart_ca)
'''

