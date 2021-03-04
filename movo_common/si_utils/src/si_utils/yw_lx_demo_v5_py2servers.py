# Python3 Interfaces for ARC actions libraries - Intelligent Human-Robot Collaboration System
# It connects to py2 servers, with py3 clients
# Email: yiwen.chen@u.nus.edu
import rospy

import rospy
import actionlib
import numpy as np
import moveit_commander
import time
import threading

# msgs
from actionlib_msgs.msg import GoalStatus
import movo_arc_lib.msg as arclib_msg
from movo_msgs.msg import JacoCartesianVelocityCmd
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasp_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from shape_msgs.msg import SolidPrimitive as sp
from geometry_msgs.msg import Pose2D, PoseStamped, Quaternion, Vector3
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import moveit_msgs.msg
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from si_utils.head_jtas_test import HeadJTASTest
from si_utils.base_motion_test import BaseMotionTest
from si_utils.voice_test import MovoVoiceTest

# interfaces and clients
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)

from movo_action_clients.gripper_action_client import GripperActionClient
from movo_action_clients.move_base_action_client import MoveBaseActionClient
from movo_action_clients.torso_action_client import TorsoActionClient

import copy

from movo_action_clients.gripper_action_client import GripperActionClient

from tf.transformations import *

class ref_Jaco_JointControl_JointTrajFollow(object):
    def __init__(self, arm, dof='7dof'):
        self.arm=arm
        self._client = actionlib.SimpleActionClient(
            'movo/%s_arm_controller/follow_joint_trajectory' % arm,
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self.dof = dof
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def get_action(self):
        return self._client
    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy.copy(positions)
        point.velocities = [0.0] * len(self._goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def add_point_deg(self, joints_degree, time):
        self.add_point(map(math.radians, joints_degree), time)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time(0.0)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()
    def result_published(self):
        res=self._client.get_result()
        return type(res)!=type(None)
    def clear(self):
        arm=self.arm
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance

        if '7dof' == self.dof:
            self._goal.trajectory.joint_names = ['%s_shoulder_pan_joint' % arm,
                                                 '%s_shoulder_lift_joint' % arm,
                                                 '%s_arm_half_joint' % arm,
                                                 '%s_elbow_joint' % arm,
                                                 '%s_wrist_spherical_1_joint' % arm,
                                                 '%s_wrist_spherical_2_joint' % arm,
                                                 '%s_wrist_3_joint' % arm]
class L0_upper_jp_move_safe_srv():
    # This is a action server
    _feedback = arclib_msg.upper_jp_movo_safeFeedback()
    _result = arclib_msg.upper_jp_movo_safeResult()
    def __init__(self,arclib_node):
        self._c = arclib_node
        self._action_name = "L0_upper_jp_move_safe"
        self._as = actionlib.SimpleActionServer(self._action_name,arclib_msg.upper_jp_movo_safeAction,
                                              execute_cb=self.execute_cb,
                                              auto_start=False)
        self._as.register_preempt_callback(self.preempt_cb)

        self._as.start()
        print(self._action_name,"Started!")

    def _goal_check(self,goal):
        # fake check, not finished yet. Bug Here
        # TODO not finished yet. Not important.
        # for i in check_list:
        #     pi=3.1415
        #     if(i<pi and i > -pi):
        #         pass
        #     else:
        #         return False
        return True
    def _pub_feedback(self):
        self._feedback.duration=time.time()-self.goal_start_time
        self._feedback.jp_right=self._c.E0_get_right_jointstates_pos()
        self._feedback.jp_left=self._c.E0_get_left_jointstates_pos()
        self._feedback.jp_head=self._c.E0_get_head_jointstates_pos()
        self._feedback.jp_linear=self._c.E0_get_linear_jointstate_pos()
        self._feedback.r_current_force=self._c.E0_get_r_cart_force()
        self._feedback.l_current_force=self._c.E0_get_l_cart_force()
        self._as.publish_feedback(self._feedback)
    @staticmethod
    def _get_jps_from_goal(goal):
        return tuple(goal.jp_right)+tuple(goal.jp_left)+tuple([goal.jp_linear])+tuple(goal.jp_head)
    def cancel_all_sub_goals(self):
        # TODO it needs some time to publish cancel the goal
        freq = rospy.Rate(10)
        freq.sleep()
        self._c.move_group.get_move_action().cancel_all_goals()

    def preempt_cb(self):
        print("This action has been preempted")
        # cancel all sub goals
        self.cancel_all_sub_goals()

    def execute_cb(self,goal):
        success = True
        rospy.loginfo("goal get"+str(goal))
        self.goal_start_time=time.time()
        if(not(self._goal_check(goal))):
            rospy.loginfo("Fail, the goal was rejected")
            success = False

        self._pub_feedback()

        # excuting action
        bug_loop = 0
        if(success==True):

            # send sub goal to movojp server
            # for i in range(3):
            self._c.move_group.moveToJointPosition(self._c._upper_body_joints, self._get_jps_from_goal(goal), 0.001, wait=False)

            while(not rospy.is_shutdown()):

                if self._as.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self._as.set_preempted()
                    rospy.loginfo(" fail, preemted")
                    success = False
                    break

                    # print("__bug_loop",bug_loop)
                    # print("my goal=",goal)

                # update feedback and publish
                self._pub_feedback()

                # check if sub goals finished
                move_success = self._c.move_group.get_move_action().get_result()

                try:
                    done_success = move_success.error_code.val == MoveItErrorCodes.SUCCESS
                except Exception as err:
                    done_success = False
                    # print(err)
                # print(done_success)
                # print(self._c.move_group.get_move_action().get_state())
                sub_goal_state=self._c.move_group.get_move_action().get_state()
                if sub_goal_state in [GoalStatus.PREEMPTED,GoalStatus.SUCCEEDED,GoalStatus.ABORTED,GoalStatus.REJECTED,GoalStatus.RECALLED]:
                    sub_goal_done=True
                else:
                    sub_goal_done=False
                # check if force goal finished

                lmaxforce = goal.l_max_force
                rmaxforce = goal.r_max_force
                lforce = self._c.E0_get_l_cart_force()
                rforce = self._c.E0_get_r_cart_force()
                force_range_detect_left = [(abs(lmaxforce[i]) - abs(lforce[i]) < 0) for i in range(6)]
                force_range_detect_right=[(abs(rmaxforce[i])-abs(rforce[i])<0) for i in range(6)]
                force_range_detect=[force_range_detect_left[i] or force_range_detect_right[i] for i in range(6)]

                done_shut = rospy.is_shutdown()
                force_success=sum(force_range_detect)>0

                if(force_success):
                    self._as.set_preempted()
                    rospy.loginfo(" fail, preemted by force max limitation detected")
                    success = False
                    break

                # task finished, -> success
                done = done_success or done_shut or sub_goal_done
                if (done):
                    success= True
                    rospy.loginfo(" done")
                    break

                # over time exit
                if((time.time()-self.goal_start_time)>goal.duration):
                    success=False
                    rospy.loginfo(" fail, overtime")
                    break

        self._set_success_and_abort(success)

    def _set_success_and_abort(self,success):
        if success:
            self._result.success=True
            # rospy.loginfo("%s:Succeeded!",%self._action_name)
            rospy.loginfo("Succeeded!"+str(self._action_name))
            self._as.set_succeeded(self._result)
        else:
            self._result.success=False
            # rospy.loginfo("%s:Succeeded!",%self._action_name)
            rospy.loginfo("Aborted!"+str(self._action_name))
            self._as.set_aborted(self._result)

        # cancel all sub golad
        self.cancel_all_sub_goals()

    # def L0_goto_upper_body_joints_safe(self,joints,force_max=[20,20,20,10,10,10],block_until_success=True):
    #
    #     print "  -- Go To",joints
    #     for i in range(3):
    #         self.move_group.moveToJointPosition(self._upper_body_joints, joints, 0.005, wait=False)
    #     done=False
    #
    #     while(not rospy.is_shutdown()):
    #         success=self.move_group.get_move_action().get_result()
    #         try:
    #             done_success=success.error_code.val == MoveItErrorCodes.SUCCESS
    #         except Exception as err:
    #             # print(err)
    #             done_success=False
    #         # print(done_success)
    #         # self.move_group.
    #         force_range_detect_left=[(abs(force_max[i])-abs(self.cartesianforce_left[i])<0) for i in range(6)]
    #         force_range_detect_right=[(abs(force_max[i])-abs(self.cartesianforce_right[i])<0) for i in range(6)]
    #         force_range_detect=[force_range_detect_left[i] or force_range_detect_right[i] for i in range(6)]
    #         # print(force_range_detect)
    #         # print(force_range_detect_left)
    #         # print(force_range_detect_right)
    #         # print(self.cartesianforce)
    #         done_shut=rospy.is_shutdown()
    #         done_safe=sum(force_range_detect)>0
    #
    #         done=done_success or done_safe or done_shut
    #         if(done):
    #             print("done=",done_success,done_safe)
    #             self.move_group.get_move_action().cancel_all_goals()
    #             time.sleep(1)
    #             break
    #
    #     pass
class L0_dual_jp_move_safe_relate_srv():
    # This is a action server
    _feedback = arclib_msg.dual_jp_movo_safe_relateFeedback()
    _result = arclib_msg.dual_jp_movo_safe_relateResult()
    def __init__(self,arclib_node):
        self._c=arclib_node
        self._action_name="L0_dual_jp_move_safe_relate"
        self._as=actionlib.SimpleActionServer(self._action_name,
                                              arclib_msg.dual_jp_movo_safe_relateAction,
                                              execute_cb=self.execute_cb,
                                              auto_start=False)
        self.jointcontrol_client_left=ref_Jaco_JointControl_JointTrajFollow(arm="left")
        self.jointcontrol_client_right=ref_Jaco_JointControl_JointTrajFollow(arm="right")
        # self.jointcontrol_client_left=ref_Jaco_JointControl_JointTrajFollow(arm="left")
        # self.jointcontrol_client_right=ref_Jaco_JointControl_JointTrajFollow(arm="right")
        self.sub_done = False
        self._as.register_preempt_callback(self.preempt_cb)

        self._as.start()
        print(self._action_name,"Started!")

    def _goal_check(self,goal):
        return True

    def _pub_feedback(self):
        self._feedback.NotImplemented=False
        self._as.publish_feedback(self._feedback)
        pass

    def cancel_all_sub_goals(self):
        # TODO it needs some time to publish cancel the goal
        freq = rospy.Rate(10)
        freq.sleep()
        # self._c.move_group.get_move_action().cancel_all_goals()
        self.jointcontrol_client_left.get_action().cancel_all_goals()
        self.jointcontrol_client_right.get_action().cancel_all_goals()

    def preempt_cb(self):
        print("This action has been preempted")
        self.cancel_all_sub_goals()

    def _get_jps_from_goal(self,goal):
        pass

    def L0_dual_joint_rot_relate(self, l_rot, r_rot, total_time):
        self.sub_done = False
        point_num=int(total_time*10)
        # self.L0_dual_joint_rot_relate_done = False
        self.jointcontrol_client_left.clear()
        self.jointcontrol_client_right.clear()

        # l_rot=[0,0,0,0,0,0,0.3]
        # r_rot=[0,0,0,0,0,0,0.3]
        # left_rot=[0,3.1415/6,0]

        total_time = float(total_time)
        tmp_r = rospy.wait_for_message("/movo/right_arm/joint_states", JointState)
        tmp_l = rospy.wait_for_message("/movo/left_arm/joint_states", JointState)
        current_angles_r = tmp_r.position  # tuple (p1,p2,p3,p4,p5,p6,p7) in radius -pi -> pi
        current_angles_l = tmp_l.position
        self.jointcontrol_client_left.add_point(current_angles_l, 0)
        self.jointcontrol_client_right.add_point(current_angles_r, 0)

        du = 0.
        dt_r = total_time / float(point_num)

        while (du < total_time):
            du += dt_r
            l_jp_position = list(current_angles_l)
            r_jp_position = list(current_angles_r)
            # for i in range()
            l_jp_position = [l_jp_position[i] + l_rot[i] * du / total_time for i in range(len(l_jp_position))]
            r_jp_position = [r_jp_position[i] + r_rot[i] * du / total_time for i in range(len(r_jp_position))]
            self.jointcontrol_client_left.add_point(l_jp_position, du)
            self.jointcontrol_client_right.add_point(r_jp_position, du)
        print("lrot,",l_rot)
        print("rrot,",r_rot)
        self.jointcontrol_client_right.start()
        self.jointcontrol_client_left.start()
        # self.jointcontrol_client_right.wait(total_time * 2)
        # self.jointcontrol_client_left.wait(total_time * 2)
        print("   jp relate move finsihed")
        # print("Joint Control Clinet Resut left=", self.jointcontrol_client_left._client.get_result())
        # print("Joint Control Clinet Resut right=", self.jointcontrol_client_right._client.get_result())
        self.sub_done = True

    def send_sub_goal(self,goal):
        self.sub_done=False
        self.L0_dual_joint_rot_relate(l_rot=goal.jp_left_relate,
                                      r_rot=goal.jp_right_relate,
                                      total_time=goal.duration)
        pass

    def execute_cb(self,goal):
        success=True
        rospy.loginfo("goal get"+str(goal))
        self.goal_start_time=time.time()
        if(not(self._goal_check(goal))):
            rospy.loginfo("Fail, the goal was rejected")
            success=False

        self._pub_feedback()

        if(success==True):

            self.send_sub_goal(goal)

            while(not rospy.is_shutdown()):
                r=rospy.Rate(50)
                r.sleep()
                if self._as.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self._as.set_preempted()
                    rospy.loginfo(" fail, preemted")
                    success = False
                    break

                # update feedback and publish
                self._pub_feedback()

                # check if sub goals finished
                done_success=self._c.tool_is_dualaction_success(act1=self.jointcontrol_client_left.get_action(),
                                                            act2=self.jointcontrol_client_right.get_action())
                sub_goal_done=self._c.tool_is_dualgoal_done(act1=self.jointcontrol_client_left.get_action(),
                                                            act2=self.jointcontrol_client_right.get_action())

                goal_result_published=self._c.tool_is_dualres_published(act1=self.jointcontrol_client_left.get_action(),
                                                                        act2=self.jointcontrol_client_right.get_action())
                # check if force goal finished
                done_shut=rospy.is_shutdown()
                force_success=self._c.tool_is_forcesuccess(lmaxforce=goal.l_max_force,
                                                           rmaxforce=goal.r_max_force)
                if(force_success):
                    self._as.set_preempted()
                    rospy.loginfo(" fail, preemted by force max limitation detected")
                    success = False
                    break


                # print(done_success,done_shut,sub_goal_done)
                # task finished, -> success
                # done = done_success or done_shut or sub_goal_done or self.sub_done
                done = done_shut or goal_result_published

                if (done):
                    success= True
                    rospy.loginfo(" done")
                    break

                # over time exit
                # if((time.time()-self.goal_start_time)>goal.duration):
                #     success=False
                #     rospy.loginfo(" fail, overtime"+str(self._action_name))
                #     break

            if(rospy.is_shutdown()):
                success=False
        self._set_success_and_abort(success)

    def _set_success_and_abort(self,success):
        if success:
            self._result.success=True
            # rospy.loginfo("%s:Succeeded!",%self._action_name)
            rospy.loginfo("Succeeded!"+str(self._action_name))
            self._as.set_succeeded(self._result)
        else:
            self._result.success=False
            # rospy.loginfo("%s:Succeeded!",%self._action_name)
            rospy.loginfo("Aborted!"+str(self._action_name))
            self._as.set_aborted(self._result)

        # cancel all sub golad
        self.cancel_all_sub_goals()

class L0_dual_set_gripper_srv():
    # This is a action server
    _feedback = arclib_msg.dual_set_gripperFeedback()
    _result = arclib_msg.dual_set_gripperResult()
    def __init__(self,arclib_node):
        self._c=arclib_node
        self._action_name="L0_dual_set_gripper"
        self._as=actionlib.SimpleActionServer(self._action_name,
                                              arclib_msg.dual_set_gripperAction,
                                              execute_cb=self.execute_cb,
                                              auto_start=False)


        self._lgripper = GripperActionClient('left')
        self._rgripper = GripperActionClient('right')

        self._gripper_closed = 0.00
        self._gripper_open = 0.165

        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()
        print(self._action_name,"Started!")

    def _goal_check(self,goal):
        # fake check, not finished yet. Bug Here
        # TODO not finished yet.
        # for i in check_list:
        #     pi=3.1415
        #     if(i<pi and i > -pi):
        #         pass
        #     else:
        #         return False
        return True
    def _pub_feedback(self):
        self._feedback.NotImplemented=False
        self._as.publish_feedback(self._feedback)
        pass

    def cancel_all_sub_goals(self):
        # TODO it needs some time to publish cancel the goal
        freq = rospy.Rate(5)
        freq.sleep()
        # self._c.move_group.get_move_action().cancel_all_goals()
        self._rgripper._client.cancel_all_goals()
        self._lgripper._client.cancel_all_goals()

    def preempt_cb(self):
        print("This action has been preempted")
        # cancel all sub goals
        self.cancel_all_sub_goals()

    def _get_jps_from_goal(self,goal):
        pass


    def send_sub_goal(self,goal):
        # print "do close gripper"
        value=goal.value
        v = float(value) * (self._gripper_open - self._gripper_closed) + self._gripper_closed
        self._lgripper.command(v, block=False)
        self._rgripper.command(v, block=False)

    def execute_cb(self,goal):
        success=True
        rospy.loginfo("goal get"+str(goal))
        self.goal_start_time=time.time()
        if(not(self._goal_check(goal))):
            rospy.loginfo("Fail, the goal was rejected")
            success=False

        self._pub_feedback()

        if(success==True):

            self.send_sub_goal(goal)
            while(not rospy.is_shutdown()):

                if self._as.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self._as.set_preempted()
                    rospy.loginfo(" fail, preemted")
                    success = False
                    break

                # update feedback and publish
                self._pub_feedback()

                # both sub goal finished
                done_success=self._c.tool_is_dualaction_success(act1=self._lgripper._client,act2=self._rgripper._client)
                sub_goal_done=self._c.tool_is_dualgoal_done(act1=self._lgripper._client,act2=self._rgripper._client)

                # task finished, -> success
                done_shut=rospy.is_shutdown()
                # print(done_success,sub_goal_done)
                done = done_success or sub_goal_done or done_shut

                if (done):
                    success= True
                    rospy.loginfo(" done")
                    break

            if(rospy.is_shutdown()):
                success=False
        self._set_success_and_abort(success)

    def _set_success_and_abort(self,success):
        if success:
            self._result.success=True
            # rospy.loginfo("%s:Succeeded!",%self._action_name)
            rospy.loginfo("Succeeded!"+str(self._action_name))
            self._as.set_succeeded(self._result)
        else:
            self._result.success=False
            # rospy.loginfo("%s:Succeeded!",%self._action_name)
            rospy.loginfo("Aborted!"+str(self._action_name))
            self._as.set_aborted(self._result)

        # cancel all sub golad
        self.cancel_all_sub_goals()
class L0_dual_task_move_safe_relate_srv():
    # This is a action server
    _feedback = arclib_msg.dual_task_move_safe_relateFeedback()
    _result = arclib_msg.dual_task_move_safe_relateResult()
    def __init__(self,arclib_node):
        self._c=arclib_node
        self._action_name="L0_dual_task_move_safe_relate"
        self._as=actionlib.SimpleActionServer(self._action_name,
                                              arclib_msg.dual_task_move_safe_relateAction,
                                              execute_cb=self.execute_cb,
                                              auto_start=False)

        self.jointcontrol_client_left=ref_Jaco_JointControl_JointTrajFollow(arm="left")
        self.jointcontrol_client_right=ref_Jaco_JointControl_JointTrajFollow(arm="right")
        # self.jointcontrol_client_left=ref_Jaco_JointControl_JointTrajFollow(arm="left")
        # self.jointcontrol_client_right=ref_Jaco_JointControl_JointTrajFollow(arm="right")

        self._as.register_preempt_callback(self.preempt_cb)

        self._as.start()
        print(self._action_name,"Started!")

    def _goal_check(self,goal):
        return True
    def _pub_feedback(self):
        self._feedback.NotImplemented=False
        self._as.publish_feedback(self._feedback)
        pass

    def cancel_all_sub_goals(self):
        # TODO it needs some time to publish cancel the goal
        freq = rospy.Rate(10)
        freq.sleep()
        # self._c.move_group.get_move_action().cancel_all_goals()
        self.jointcontrol_client_left.get_action().cancel_all_goals()
        self.jointcontrol_client_right.get_action().cancel_all_goals()

    def preempt_cb(self):
        print("This action has been preempted")
        self.cancel_all_sub_goals()

    def _get_jps_from_goal(self,goal):
        pass

    def L0_dual_tcp_move_relate(self,lmove,rmove,total_time,point_num=10):
        point_num=int(point_num)
        self.jointcontrol_client_left.clear()
        self.jointcontrol_client_right.clear()
        total_time=float(total_time)
        lwaypoints = []
        lpose = self._c.movegroup_larm_group.get_current_pose().pose
        dz = lmove[2]/point_num
        dy = lmove[1]/point_num
        dx = lmove[0]/point_num
        for i in range(point_num):
            lpose.position.x += dx
            lpose.position.y += dy
            lpose.position.z += dz  # First move up (z)
            lwaypoints.append(copy.deepcopy(lpose))

        rwaypoints=[]
        rpose = self._c.movegroup_rarm_group.get_current_pose().pose
        dz = rmove[2]/point_num
        dy = rmove[1]/point_num
        dx = rmove[0]/point_num
        for i in range(point_num):
            rpose.position.x += dx
            rpose.position.y += dy
            rpose.position.z += dz  # First move up (z)
            rwaypoints.append(copy.deepcopy(rpose))

        target_lpose=[lpose.position.x,lpose.position.y,lpose.position.z]
        target_rpose=[rpose.position.x,rpose.position.y,rpose.position.z]
        self.dual_tcp_move_relate_target_lpose = target_lpose
        self.dual_tcp_move_relate_target_rpose = target_rpose

        (lplan, lfraction) = self._c.movegroup_larm_group.compute_cartesian_path(
            lwaypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        (rplan, rfraction) = self._c.movegroup_rarm_group.compute_cartesian_path(
            rwaypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        tmp_r = rospy.wait_for_message("/movo/right_arm/joint_states", JointState)
        tmp_l = rospy.wait_for_message("/movo/left_arm/joint_states", JointState)
        current_angles_r = tmp_r.position
        current_angles_l = tmp_l.position
        self.jointcontrol_client_left.add_point(current_angles_l,0)
        self.jointcontrol_client_right.add_point(current_angles_r,0)

        du=0
        dt_r=total_time/float(len(rplan.joint_trajectory.points))

        for jp in rplan.joint_trajectory.points:
            # print(jp)
            jp_position=list(jp.positions)
            self.jointcontrol_client_right.add_point(jp_position,du)
            du += dt_r

        du = 0
        dt_l = total_time / len(lplan.joint_trajectory.points)
        for jp in lplan.joint_trajectory.points:
            # print(jp)
            jp_position = list(jp.positions)
            self.jointcontrol_client_left.add_point(jp_position, du)
            du += dt_l

        self.jointcontrol_client_right.start()
        self.jointcontrol_client_left.start()
        pass

    def send_sub_goal(self,goal):

        self.L0_dual_tcp_move_relate(
            lmove=goal.pos_l,
            rmove=goal.pos_r,
            total_time=goal.time,
            point_num=goal.time*8)

    def execute_cb(self,goal):
        success=True
        rospy.loginfo("goal get"+str(goal))
        self.goal_start_time=time.time()
        if(not(self._goal_check(goal))):
            rospy.loginfo("Fail, the goal was rejected")
            success=False

        self._pub_feedback()

        if(success==True):

            self.send_sub_goal(goal)

            while(not rospy.is_shutdown()):
                r=rospy.Rate(50)
                r.sleep()
                if self._as.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self._as.set_preempted()
                    rospy.loginfo(" fail, preemted")
                    success = False
                    break

                # update feedback and publish
                self._pub_feedback()

                # check if sub goals finished
                # move_success = self._c.move_group.get_move_action().get_result()


                done_success=self._c.tool_is_dualaction_success(act1=self.jointcontrol_client_left.get_action(),
                                                            act2=self.jointcontrol_client_right.get_action())
                sub_goal_done=self._c.tool_is_dualgoal_done(act1=self.jointcontrol_client_left.get_action(),
                                                            act2=self.jointcontrol_client_right.get_action())
                # check if force goal finished
                done_shut=rospy.is_shutdown()
                force_success=self._c.tool_is_forcesuccess(lmaxforce=goal.l_max_force,
                                                           rmaxforce=goal.r_max_force)
                if(force_success):
                    self._as.set_preempted()
                    rospy.loginfo(" fail, preemted by force max limitation detected")
                    success = False
                    break

                # task finished, -> success
                done = done_success or done_shut or sub_goal_done

                if (done):
                    success= True
                    rospy.loginfo(" done")
                    break

                # over time exit
                # if((time.time()-self.goal_start_time)>goal.duration):
                #     success=False
                #     rospy.loginfo(" fail, overtime"+str(self._action_name))
                #     break

            if(rospy.is_shutdown()):
                success=False
        self._set_success_and_abort(success)

    def _set_success_and_abort(self,success):
        if success:
            self._result.success=True
            # rospy.loginfo("%s:Succeeded!",%self._action_name)
            rospy.loginfo("Succeeded!"+str(self._action_name))
            self._as.set_succeeded(self._result)
        else:
            self._result.success=False
            # rospy.loginfo("%s:Succeeded!",%self._action_name)
            rospy.loginfo("Aborted!"+str(self._action_name))
            self._as.set_aborted(self._result)

        # cancel all sub golad
        self.cancel_all_sub_goals()

class L0_dual_task_move_safe_srv():
   # This is a action server
    _feedback = arclib_msg.dual_task_move_safeFeedback()
    _result = arclib_msg.dual_task_move_safeResult()
    def __init__(self, arclib_node):
        self._c = arclib_node
        self._action_name = "L0_dual_task_move_safe"
        self._as = actionlib.SimpleActionServer(self._action_name, arclib_msg.dual_task_move_safeAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()
        print(self._action_name, "Started!")


    @staticmethod
    def r_to_q(r_list, q_org):
        rx, ry, rz = r_list
        q_rot = quaternion_from_euler(rx, ry, rz)
        q_new = quaternion_multiply(q_rot, q_org)
        return q_new

    def L0_arm_move(self,arm,pos,orn,relate_frame,tolerance=0.005,wait=True):
        assert arm in ["left","right"]
        target = PoseStamped()
        # target.header.frame_id = arm+"_ee_link"
        # target.header.frame_id = "base_link"
        target.header.frame_id = relate_frame
        target.pose.position.x = pos[0]
        target.pose.position.y = pos[1]
        target.pose.position.z = pos[2]
        q_org = [0, 0, 0, 1]
        q_new = self.r_to_q(orn, q_org)
        target.pose.orientation = Quaternion(q_new[0], q_new[1], q_new[2], q_new[3])
        if arm=="left":
            self._c.lmove_group.moveToPose(target, "left_ee_link", tolerance = tolerance,wait=wait)
        elif arm == "right":
            self._c.rmove_group.moveToPose(target, "right_ee_link", tolerance=tolerance, wait=wait)
    def send_sub_goals(self,goal):
        pos_r=goal.pos_r
        pos_l=goal.pos_l
        orn_r=goal.orn_r
        orn_l=goal.orn_l
        t=goal.time
        self.L0_arm_move("left",pos=pos_l,orn=orn_l,relate_frame="base_link")
        self.L0_arm_move("right",pos=pos_r,orn=orn_r,relate_frame="base_link")
    def execute_cb(self, goal):
        success = True
        rospy.loginfo("goal get" + str(goal))
        self.goal_start_time = time.time()
        if (not (self._goal_check(goal))):
            rospy.loginfo("Fail, the goal was rejected")
            success = False

        self._pub_feedback()


        if (success == True):

            self._c.move_group.moveToJointPosition(self._c._upper_body_joints, self._get_jps_from_goal(goal), 0.001,
                                                   wait=False)

            while (not rospy.is_shutdown()):

                if self._as.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self._as.set_preempted()
                    rospy.loginfo(" fail, preemted")
                    success = False
                    break

                    # print("__bug_loop",bug_loop)
                    # print("my goal=",goal)

                # update feedback and publish
                self._pub_feedback()

                # check if sub goals finished
                move_success = self._c.move_group.get_move_action().get_result()

                try:
                    done_success = move_success.error_code.val == MoveItErrorCodes.SUCCESS
                except Exception as err:
                    done_success = False
                    # print(err)
                # print(done_success)
                # print(self._c.move_group.get_move_action().get_state())
                sub_goal_state = self._c.move_group.get_move_action().get_state()
                if sub_goal_state in [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                                      GoalStatus.REJECTED, GoalStatus.RECALLED]:
                    sub_goal_done = True
                else:
                    sub_goal_done = False
                # check if force goal finished

                lmaxforce = goal.l_max_force
                rmaxforce = goal.r_max_force
                lforce = self._c.E0_get_l_cart_force()
                rforce = self._c.E0_get_r_cart_force()
                force_range_detect_left = [(abs(lmaxforce[i]) - abs(lforce[i]) < 0) for i in range(6)]
                force_range_detect_right = [(abs(rmaxforce[i]) - abs(rforce[i]) < 0) for i in range(6)]
                force_range_detect = [force_range_detect_left[i] or force_range_detect_right[i] for i in range(6)]

                done_shut = rospy.is_shutdown()
                force_success = sum(force_range_detect) > 0

                if (force_success):
                    self._as.set_preempted()
                    rospy.loginfo(" fail, preemted by force max limitation detected")
                    success = False
                    break

                # task finished, -> success
                done = done_success or done_shut or sub_goal_done
                if (done):
                    success = True
                    rospy.loginfo(" done")
                    break

                # over time exit
                if ((time.time() - self.goal_start_time) > goal.duration):
                    success = False
                    rospy.loginfo(" fail, overtime")
                    break

        self._set_success_and_abort(success)
    def _goal_check(self, goal):
        # TODO not finished yet. Not important.
        return True
    def _pub_feedback(self):
        self._as.publish_feedback(self._feedback)
    def cancel_all_sub_goals(self):

        freq = rospy.Rate(10)
        freq.sleep()
        self._c.move_group.get_move_action().cancel_all_goals()
    def preempt_cb(self):
        print("This action has been preempted")
        # cancel all sub goals
        self.cancel_all_sub_goals()
    def _set_success_and_abort(self, success):
            if success:
                self._result.success = True
                # rospy.loginfo("%s:Succeeded!",%self._action_name)
                rospy.loginfo("Succeeded!" + str(self._action_name))
                self._as.set_succeeded(self._result)
            else:
                self._result.success = False
                # rospy.loginfo("%s:Succeeded!",%self._action_name)
                rospy.loginfo("Aborted!" + str(self._action_name))
                self._as.set_aborted(self._result)

            # cancel all sub golad
            self.cancel_all_sub_goals()


class ARC_ACTION_LIB_NODE():
    def __init__(self):
        rospy.init_node("ARC_ACTION_LIB_NODE")

        self.bug_ignore=True

        # init movo groups
        self._init_movo_groups()

        # update sensors data
        self._init_start_update_sensors()

        # init servers
        self._init_all_servers()

        # init settings
        self._init_settings()

        rospy.spin()
    def _init_settings(self):
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
    def E0_get_jointstates_pos(self):
        return self.jointstates_pos
    def E0_get_left_jointstates_pos(self):
        return self.left_arm_js_pos
    def E0_get_right_jointstates_pos(self):
        return self.right_arm_js_pos
    def E0_get_head_jointstates_pos(self):
        return self.head_js_pos
    def E0_get_linear_jointstate_pos(self):
        return self.linear_js_pos
    def E0_get_left_cartesian_pos(self):
        return self.movegroup_larm_group.get_current_pose()
    def E0_get_right_cartesian_pos(self):
        return self.movegroup_rarm_group.get_current_pose()
    def E0_get_tcp_pose_left(self):
        lpose = self.movegroup_larm_group.get_current_pose().pose
        return lpose
    def E0_get_tcp_pose_right(self):
        rpose = self.movegroup_rarm_group.get_current_pose().pose
        return rpose
    def E0_get_l_cart_force(self):
        return self.cartesianforce_left
    def E0_get_r_cart_force(self):
        return self.cartesianforce_right
    def tool_is_res_published(self,act):
        return type(act.get_result())!=type(None)
    def tool_is_dualres_published(self,act1,act2):
        p1=self.tool_is_res_published(act1)
        p2=self.tool_is_res_published(act2)
        return p1 and p2
    def tool_is_goal_done(self,act):

        goal_state = act.get_state()
        done_status_list = [
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED]

        if (goal_state in done_status_list):
            sub_goal_done = True
            # print("sub_goal_done, status =", sub_goal_done)
        else:
            sub_goal_done = False
        return sub_goal_done
    def tool_is_dualgoal_done(self,act1,act2):

        sub_goal_done_l = self.tool_is_goal_done(act=act1)
        sub_goal_done_r = self.tool_is_goal_done(act=act2)
        sub_goal_done = sub_goal_done_l and sub_goal_done_r
        return sub_goal_done
    def tool_is_dualaction_success(self,act1,act2):
        left_move_success = act1.get_result()
        right_move_success = act2.get_result()
        try:
            right_done_success = right_move_success.error_code.val == MoveItErrorCodes.SUCCESS
            left_done_success = left_move_success.error_code.val == MoveItErrorCodes.SUCCESS
            done_success = right_done_success and left_done_success
        except Exception as err:
            done_success = False
        return done_success
    def tool_is_forcesuccess(self,lmaxforce,rmaxforce):

        # lmaxforce = goal.l_max_force
        # rmaxforce = goal.r_max_force
        lforce = self.E0_get_l_cart_force()
        rforce = self.E0_get_r_cart_force()
        force_range_detect_left = [(abs(lmaxforce[i]) - abs(lforce[i]) < 0) for i in range(6)]
        force_range_detect_right = [(abs(rmaxforce[i]) - abs(rforce[i]) < 0) for i in range(6)]
        force_range_detect = [force_range_detect_left[i] or force_range_detect_right[i] for i in range(6)]
        done_shut = rospy.is_shutdown()
        force_success = sum(force_range_detect) > 0
        return force_success

    def _init_movo_groups(self):


        self.move_group = MoveGroupInterface("upper_body", "base_link")
        self.lmove_group = MoveGroupInterface("left_arm", "base_link")
        self.rmove_group = MoveGroupInterface("right_arm", "base_link")
        self.move_group.setPlannerId("RRTConnectkConfigDefault")
        self.lmove_group.setPlannerId("RRTConnectkConfigDefault")
        self.rmove_group.setPlannerId("RRTConnectkConfigDefault")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_names = self.robot.get_group_names()
        print("Avaliable Groups_name",group_names)

        self.movegroup_upper_group = moveit_commander.MoveGroupCommander("upper_body")
        self.movegroup_rarm_group = moveit_commander.MoveGroupCommander("right_arm")
        self.movegroup_larm_group = moveit_commander.MoveGroupCommander("left_arm")


        # # time.sleep(15)
        # # time.sleep(15)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)  # For trajectory publish in rviz display

        print("All groups are inited!")
    def _init_all_servers(self):
        self.Server_L0_upper_jp_move_safe = L0_upper_jp_move_safe_srv(self)
        self.Server_L0_dual_jp_move_safe_relate = L0_dual_jp_move_safe_relate_srv(self)
        self.Server_L0_dual_task_move_safe_relate = L0_dual_task_move_safe_relate_srv(self)
        self.Server_L0_dual_set_gripper=L0_dual_set_gripper_srv(self)
    def _init_start_update_sensors(self):
        self._subscribe_force("right")
        self._subscribe_force("left")
        self._subscribe_jointstates()

        self.cartesianforce_right = None
        self.cartesianforce_left = None
        self.jointstates_pos = None
        self.jointstates_vel = None
        self.jointstates_effort = None
        self.right_arm_js_pos = None
        self.left_arm_js_pos = None
        self.right_arm_js_vel = None
        self.left_arm_js_vel = None
        self.head_js_pos = None
        self.head_js_vel = None
        self.linear_js_pos = None
        self.linear_js_vel = None
        # When you add new states, please regist in _states
            # Wait till all states got updated
        while(sum(type(i)==type(None) for i in self._states)):
            print(self._states)
            time.sleep(1)
            # print(self.jointstates_effort)
            print(" Wait ... till all states got first updated")
        print(" All States Got Updated at the first time!")
    @property
    def _states(self):
        return [
            self.cartesianforce_left,
            self.cartesianforce_right,
            self.jointstates_pos,
            self.jointstates_vel,
            self.jointstates_effort,
            self.right_arm_js_pos,
            self.left_arm_js_pos,
            self.right_arm_js_vel,
            self.left_arm_js_vel,
            self.head_js_pos,
            self.head_js_vel,
            self.linear_js_pos,
            self.linear_js_vel
        ]
    def _subscribe_force_callback_right(self, data):
        fx = data.x
        fy = data.y
        fz = data.z
        fr = data.theta_x
        fp = data.theta_y
        fa = data.theta_z
        # print data
        self.cartesianforce_right = [fx, fy, fz, fr, fp, fa]
    def _subscribe_force_callback_left(self, data):
        fx = data.x
        fy = data.y
        fz = data.z
        fr = data.theta_x
        fp = data.theta_y
        fa = data.theta_z
        # print data
        self.cartesianforce_left = [fx, fy, fz, fr, fp, fa]
        # print(self.cartesianforce)
    def _subscribe_force(self, arm):
        assert arm in ["left", "right"]
        arm_name = arm+'_arm'
        # print "======== I'm subscribe_force =========="
        if(arm=="left"):
            rospy.Subscriber("/movo/{}/cartesianforce".format(arm_name), JacoCartesianVelocityCmd,
                             self._subscribe_force_callback_left)
        elif(arm=="right"):
            rospy.Subscriber("/movo/{}/cartesianforce".format(arm_name), JacoCartesianVelocityCmd,
                             self._subscribe_force_callback_right)
    def _subscribe_jointstates_callback(self,data):
        # [linear_joint, pan_joint, tilt_joint,
        # mid_body_joint,
        # right_shoulder_pan_joint, right_shoulder_lift_joint,right_arm_half_joint,
        # right_elbow_joint, right_wrist_spherical_1_joint, right_wrist_spherical_2_joint,
        #  right_wrist_3_joint,
        #  left_shoulder_pan_joint, left_shoulder_lift_joint, left_arm_half_joint,
        #  left_elbow_joint, left_wrist_spherical_1_joint, left_wrist_spherical_2_joint,
        #  left_wrist_3_joint,
        #  right_gripper_finger1_joint, right_gripper_finger2_joint, right_gripper_finger3_joint,
        #  left_gripper_finger1_joint, left_gripper_finger2_joint, left_gripper_finger3_joint]

        self.jointstates_pos = data.position
        self.jointstates_vel = data.velocity
        self.jointstates_effort = data.effort

        self.linear_js_pos = data.position[0:1][0]
        self.linear_js_vel = data.velocity[0:1][0]

        self.head_js_pos = data.position[1:3] #
        self.head_js_vel = data.velocity[1:3] #

        self.right_arm_js_pos = data.position[4:11]
        self.right_arm_js_vel = data.velocity[4:11]

        self.left_arm_js_pos = data.position[11:18]
        self.left_arm_js_vel = data.velocity[11:18]
    def _subscribe_jointstates(self):
        rospy.Subscriber("/joint_states", JointState,
                         self._subscribe_jointstates_callback)

if __name__=="__main__":
    print("Start")
    arc=ARC_ACTION_LIB_NODE()