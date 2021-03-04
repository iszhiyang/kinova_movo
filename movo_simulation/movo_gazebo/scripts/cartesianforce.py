#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool

import moveit_commander
from movo_msgs.msg import JacoCartesianVelocityCmd



right_arm = moveit_commander.MoveGroupCommander("right_arm")
left_arm = moveit_commander.MoveGroupCommander("left_arm")

global rf
global lf
def subscribe_force_callback_right(data):
    # print("========")
    # print(data)
    global rf
    rf = JacoCartesianVelocityCmd()
    rf.x = data.wrench.force.x
    rf.y = data.wrench.force.y
    rf.z = data.wrench.force.z
    rf.theta_x = data.wrench.torque.x
    rf.theta_y = data.wrench.torque.y
    rf.theta_z = data.wrench.torque.z


def subscribe_force_callback_left(data):
    global lf
    lf = JacoCartesianVelocityCmd()
    lf.x = data.wrench.force.x
    lf.y = data.wrench.force.y
    lf.z = data.wrench.force.z
    lf.theta_x = data.wrench.torque.x
    lf.theta_y = data.wrench.torque.y
    lf.theta_z = data.wrench.torque.z


def get_force():
    rospy.Subscriber("/sim/right_arm/cartesianforce", WrenchStamped, subscribe_force_callback_right)
    rate.sleep()
    rospy.Subscriber("/sim/left_arm/cartesianforce", WrenchStamped, subscribe_force_callback_left)
    rate.sleep()

    global rf
    global lf

    pub_r = rospy.Publisher('/movo/right_arm/cartesianforce', JacoCartesianVelocityCmd, queue_size=10)
    pub_l = rospy.Publisher('/movo/left_arm/cartesianforce', JacoCartesianVelocityCmd, queue_size=10)

    while not rospy.is_shutdown():
        pub_r.publish(rf)
        rate.sleep()
        pub_l.publish(lf)
        rate.sleep()
    


if __name__ == "__main__":
    # Create a node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("lx_cartesianforce")
    rate = rospy.Rate(10)
    rospy.wait_for_message('/sim_initialized',Bool)    

    get_force() 

    rospy.spin()