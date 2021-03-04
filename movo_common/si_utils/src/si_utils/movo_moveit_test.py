#!/usr/bin/env python

"""--------------------------------------------------------------------
Copyright (c) 2017, Kinova Robotics inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.
      
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

--------------------------------------------------------------------------"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
# from gripper_action_test import GripperActionTest
from movo_action_clients.gripper_action_client import GripperActionClient
from moveit_python import MoveGroupInterface
from movo_action_clients.torso_action_client import TorsoActionClient
from sensor_msgs.msg import JointState

def movo_moveit_test():
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## ^^^^^
    ## CALL_SUB_TUTORIAL imports
    ##
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('movo_moveit_test',
                  anonymous=True)
    
    # is_sim = rospy.get_param('~sim',False)
    is_sim = True

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()
    names = robot.get_group_names()


    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    
    
    ##Gripper action clients
    # lgripper = GripperActionTest('left')
    # rgripper = GripperActionTest('right')
    
    lgripper = GripperActionClient('left')
    rgripper = GripperActionClient('right')


    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    # group = moveit_commander.MoveGroupCommander("upper_body")
    print "============ Hi! Available Robot Groups:"
    group_names =  robot.get_group_names()
    rospy.loginfo(group_names)
    rospy.sleep(1.0)

    # rospy.loginfo("======upper_body_group connected =========")
    larm_group = moveit_commander.MoveGroupCommander("left_arm")
    rospy.loginfo("======l_arm move_group connected =========")
    larm_pos = larm_group.get_current_pose()
    



    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,queue_size=10)

    rospy.sleep(2)
    # scene.remove_world_object("floor")
    
    # publish a demo scene
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.0
    p.pose.position.y = 0.0
    p.pose.position.z = -0.01
    p.pose.orientation.w = 1.0
    scene.add_box("floor", p, (4.0, 4.0, 0.02))
    

    ## We can get a list of all the groups in the robot
        

    # name_lg = group_names[2]
    # group_lg = moveit_commander.MoveGroupCommander("left_gripper")
    # frame_lg = group_lg.get_planning_frame()
    # rospy.loginfo("========== Gripper frame is: =========")
    # rospy.loginfo(frame_lg)
    
    ## Sometimes for debugging it is useful to print the entire state of the
    ## robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    
    group.set_planner_id("RRTConnectkConfigDefault")

    
    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector
    if (True == is_sim):
        gripper_closed = 0.96
        gripper_open = 0.0
    else:
        gripper_closed = 0.0
        gripper_open = 0.165        

    
    
    def test_gripper():
        raw_input("========== Press Enter to test Gripper:")
        rospy.loginfo(lgripper.result)
        lgripper.command(0.085, True)
        rgripper.command(0.085, True)
        lgripper.wait()
        rgripper.wait()

        lgripper.command(0.165)
        rgripper.command(0.165)
        lgripper.wait()
        rgripper.wait()
        curr_pos = lgripper.result().position  # 0.1558
        rospy.loginfo("=========== Current Gripper position is:")
        rospy.loginfo(curr_pos)

        raw_input("====== Press Enter to close Gripper:")
        lgripper.command(0.0, block=True, timeout=3)
        rgripper.command(0.0)
        # lgripper.wait()
        # rgripper.wait()
    

    def test_named_pos():
        raw_input("========== Press Enter to test named targets: home")
        group.set_named_target("homed")
        plan = group.plan()
        group.execute(plan)
        lgripper.command(gripper_open)
        rgripper.command(gripper_open)
        lgripper.wait()
        rgripper.wait()
        
        rospy.sleep(2.0) 
        
        group.set_named_target("tucked")
        plan = group.plan()
        group.execute(plan)
        lgripper.command(gripper_closed)
        rgripper.command(gripper_closed)
        lgripper.wait()
        rgripper.wait()

        rospy.sleep(2.0) 
        '''
        group.set_named_target("pos1")
        plan = group.plan()
        group.execute(plan)
        lgripper.command(gripper_open)
        rgripper.command(gripper_open)
        lgripper.wait()
        rgripper.wait()

        rospy.sleep(2.0) 
        
        group.set_named_target("tucked")
        plan = group.plan()
        group.execute(plan)
        lgripper.command(gripper_closed)
        rgripper.command(gripper_closed)
        lgripper.wait()
        rgripper.wait()    
        '''
        ## When finished shut down moveit_commander.
    

    test_gripper()
    # test_named_pos()

    raw_input("======== Enter to disconnect ros:")
    moveit_commander.roscpp_shutdown()

    ## END_TUTORIAL

    print "============ STOPPING"

def torso_test():
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('movo_moveit_test',
                  anonymous=True)
    movo_torsor = TorsoActionClient()
    movo_torsor.clear()
    temp_torso = rospy.wait_for_message("/movo/linear_actuator/joint_states", JointState)
    current_torso_pos = list(temp_torso.position)
    movo_torsor.add_point(current_torso_pos, 0.0)
    movo_torsor.add_point([0.40], 4)
    movo_torsor.start()
    movo_torsor.wait(14.0)
    print "Movo Completing Torso Motion"



if __name__=='__main__':
  try:
    #  movo_moveit_test()
    torso_test()

  except rospy.ROSInterruptException:
    pass

