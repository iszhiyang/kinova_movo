#!/usr/bin/env python

import copy
import actionlib
import rospy
import sys
import tf
import moveit_commander
import math

from si_utils.lx_transformerROS import my_transformer
from tf.transformations import *
from moveit_msgs.msg import Constraints, OrientationConstraint

from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from movo_action_clients.gripper_action_client import GripperActionClient
from movo_action_clients.move_base_action_client import MoveBaseActionClient
from movo_action_clients.torso_action_client import TorsoActionClient
from sensor_msgs.msg import JointState

from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasp_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from shape_msgs.msg import SolidPrimitive as sp
from geometry_msgs.msg import Pose2D,PoseStamped,TransformStamped, Quaternion
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from std_msgs.msg import Bool

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        
        
        self.client = actionlib.SimpleActionClient("movo/head_controller/point_head_action", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        self.pag = PointHeadGoal()
        self.pag.max_velocity = 1.0
        self.pag.pointing_axis.x = 1.0
        self.pag.pointing_frame = "/movo_camera_link"
        

    def look_at(self, x, y, z, frame, duration=1.0):
        self.pag.target.header.stamp = rospy.get_rostime()
        self.pag.target.header.frame_id = frame
        self.pag.target.point.x = x
        self.pag.target.point.y = y
        self.pag.target.point.z = z
        self.pag.min_duration = rospy.Duration(duration)
        self.client.send_goal(self.pag)
        self.client.wait_for_result()

# Tools for grasping
class GraspingClient(object):

    def __init__(self,sim=False):
        self.scene = PlanningSceneInterface("base_link")
        self.dof = rospy.get_param('~jaco_dof')
        self.robot = moveit_commander.RobotCommander()

        self._listener = tf.TransformListener()
        # self.frame = self._listener.lookupTransform('/left_ee_link', '/base_link', rospy.Time(0))
        # rospy.loginfo("========== Left_ee to base_link is: ")
        # rospy.loginfo(self.frame)
        # curr_pos = self._listener.lookupTransform('/base_link', '/left_ee_link', rospy.Time(0))
        # rospy.loginfo(curr_pos)

        # self.move_group = moveit_commander.MoveGroupCommander("upper_body")
        # rospy.loginfo("======upper_body_group connected =========")
        # rospy.loginfo(self.move_group.get_joints())
        # self.lmove_group = moveit_commander.MoveGroupCommander("left_arm")
        # rospy.loginfo("======left_arm_group connected =========")
        # self.rmove_group = moveit_commander.MoveGroupCommander("right_arm")
        # rospy.loginfo("======right_arm_group connected =========")
        self.move_group = MoveGroupInterface("upper_body", "base_link")
        self.lmove_group = MoveGroupInterface("left_arm", "base_link")
        self.rmove_group = MoveGroupInterface("right_arm", "base_link")
        self.move_group.setPlannerId("RRTConnectkConfigDefault")
        self.lmove_group.setPlannerId("RRTConnectkConfigDefault")
        self.rmove_group.setPlannerId("RRTConnectkConfigDefault")

        if "6dof" == self.dof:
            rospy.logwarn("======= Please change launch param to 7DoF =========")

        elif "7dof" == self.dof:
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
            # flipping pose
            self.constrained_stow = [-0.23, -0.71, -1.02, -1.0, 0.9, 1.89, -2.41, 0.44, 0.71, 1.12, 1.21, -1.02, -1.84, 2.61, 0.20, 0, 0]

            # self.constrained_stow_2 = [-0.34, -0.66, -0.92, -1.18, 0.94, 1.75, -2.49, 0.54, 0.65, 1.0, 1.35, -1.07, -1.75, 2.67, 0.20, 0, 0]
            self.larm_const_stow = [2.6, -2.0, 0.0, -2.0, 0.0, 0.0, 1.0]
            self.rarm_const_stow = [-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, -1.0]
            self.tableDist = 0.8

        else:
            rospy.logerr("DoF needs to be set 6 or 7, aborting demo")
            return

        '''
        pickplace = [lg_itf, rg_itf]
        pick_result = [None, None]
        '''
        
        self.pickplace = [None]*2
        self.pickplace[0] = PickPlaceInterface("left_side", "left_gripper", verbose=True)
        self.pickplace[0].planner_id = "RRTConnectkConfigDefault"
        self.pickplace[1] = PickPlaceInterface("right_side", "right_gripper", verbose=True)
        self.pickplace[1].planner_id = "RRTConnectkConfigDefault"
        self.pick_result = [None]*2
        self._last_gripper_picked = 0
        self.place_result = [None]*2
        self._last_gripper_placed = 0
        
        self._objs_to_keep = []
        
        

        self._lgripper = GripperActionClient('left')
        self._rgripper = GripperActionClient('right')
        
        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()
        rospy.loginfo("============= FindGraspableObjectsAction connected! ============")
        
        self.scene.clear()
        
        # This is a simulation so need to adjust gripper parameters
        if sim:
            self._gripper_closed = 0.96
            self._gripper_open = 0.00
        else:
            self._gripper_closed = 0.01
            self._gripper_open = 0.165
        
            
    def add_objects_to_keep(self,obj):
        self._objs_to_keep.append(obj)
        
    def clearScene(self):
        self.scene.clear()

    def getPickCoordinates(self):

        self.updateScene(0,False)
        nut,grasps = self.getGraspableNut(False)
        bolt,grasps = self.getGraspableBolt(False)
        if (None == nut) or (None==bolt):
            rospy.loginfo("======= No nut or bolt found ========")
            return None
        center_objects = (nut.primitive_poses[0].position.y + bolt.primitive_poses[0].position.y)/2

        surface = self.getSupportSurface(nut.support_surface)
        tmp1 = surface.primitive_poses[0].position.x-surface.primitives[0].dimensions[0]/2
        surface = self.getSupportSurface(bolt.support_surface)
        tmp2 = surface.primitive_poses[0].position.x-surface.primitives[0].dimensions[0]/2
        front_edge = (tmp1+tmp2)/2
        
        coords = Pose2D(x=(front_edge-self.tableDist),y=center_objects,theta=0.0)

        return coords

    def getGBCoordinates(self):
        '''
        Get gearbox pos from arv marker
        '''
        pass



    def updateScene(self,gripper=0,plan=True):
        # find objects
        rospy.loginfo("Updating scene...")
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = plan
        goal.gripper = gripper
        # send gripper to FindGraspableObject
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        # return graspable object: objects, grasps(moveit)
        find_result = self.find_client.get_result()
        rospy.loginfo("=========== before updating, find_result.objects is: ")
        rospy.loginfo(find_result.objects)

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, True)

        # insert objects to scene
        idx = -1
        # result.object = GraspableObject = object, grasps
        # obj.object = grasp_msgs.object
        for obj in find_result.objects:
            if obj.object.primitive_poses[0].position.z < 0.5 or obj.object.primitive_poses[0].position.x > 2.0 or obj.object.primitive_poses[0].position.y > 0.5:
                continue
            idx += 1
            obj.object.name = "object%d_%d"%(idx,gripper)
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0],
                                         wait = True)

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            if obj.primitive_poses[0].position.z < 0.5:
                continue
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            obj.primitives[0].dimensions[1],  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         wait = True)

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getGraspableNut(self,planned=True):
        if self.objects == None:
            rospy.loginfo("============= No self.objects ===========")
        for obj in self.objects:
            
            # need grasps
            if len(obj.grasps) < 1 and planned:
                continue

            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5 or \
               obj.object.primitive_poses[0].position.z > 1.0 or \
               obj.object.primitive_poses[0].position.x > 2.0:
                continue
            elif (obj.object.primitives[0].type == sp.CYLINDER):
                if obj.object.primitives[0].dimensions[sp.CYLINDER_HEIGHT] < 0.102 or \
                   obj.object.primitives[0].dimensions[sp.CYLINDER_HEIGHT] > 0.142:
                    continue
            elif (obj.object.primitives[0].type == sp.BOX):
                if obj.object.primitives[0].dimensions[sp.BOX_Z] < 0.102 or \
                   obj.object.primitives[0].dimensions[sp.BOX_Z] > 0.142:
                    continue
            else:
                continue
            
            print "nut:   ",obj.object.primitive_poses[0]
           

            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getGraspableBolt(self,planned=True):
        for obj in self.objects:
            # need grasps
            if len(obj.grasps) < 1 and planned:
                continue

            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5 or \
               obj.object.primitive_poses[0].position.z > 1.0 or \
               obj.object.primitive_poses[0].position.x > 2.0:
                continue
            elif (obj.object.primitives[0].type == sp.CYLINDER):
                if obj.object.primitives[0].dimensions[sp.CYLINDER_HEIGHT] < 0.21 or \
                   obj.object.primitives[0].dimensions[sp.CYLINDER_HEIGHT] > 0.28:
                    continue
            elif (obj.object.primitives[0].type == sp.BOX):
                if obj.object.primitives[0].dimensions[sp.BOX_Z] < 0.21 or \
                   obj.object.primitives[0].dimensions[sp.BOX_Z] > 0.28:
                    continue
            else:
                continue

            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getSupportSurface(self, name):
        for surface in self.surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps, gripper=0):

        success, pick_result = self.pickplace[gripper].pick_with_retry(block.name,
                                                              grasps,
                                                              retries=1,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result[gripper] = pick_result
        self._last_gripper_picked = gripper
        return success

    def place(self, block, pose_stamped,gripper=0):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result[gripper].grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result[gripper].grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result[gripper].grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace[gripper].place_with_retry(block.name,
                                                                         places,
                                                                         retries=1,
                                                                         scene=self.scene)
        self.place_result[gripper] = place_result
        self._last_gripper_placed = gripper
        return success
    
    def goto_tuck(self):
        # remove previous objects
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(self._upper_body_joints, self.tucked, 0.05, max_velocity_scaling_factor=0.3)
            
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
            
    def goto_plan_grasp(self):
        while not rospy.is_shutdown():

            try:
                result = self.move_group.moveToJointPosition(self._upper_body_joints, self.constrained_stow, 0.01, max_velocity_scaling_factor=0.3)
            
            except:
                continue

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def goto_plan_grasp_lx(self):

        plan_lx = PoseStamped()
        plan_lx.header.frame_id = 'base_link'
        plan_lx.pose.position.x = 0.71
        plan_lx.pose.position.y = 0.435
        plan_lx.pose.position.z = 1.34

        while not rospy.is_shutdown():

            try:
                result = self.lmove_group.moveToPose(plan_lx, 'left_ee_link', 0.05)
            
            except:
                continue

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return


    def left_arm_constrained_stow(self):
        c1 = Constraints()
        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.stamp = rospy.get_rostime()
        c1.orientation_constraints[0].header.frame_id = "base_link"
        c1.orientation_constraints[0].link_name = "left_ee_link"
        c1.orientation_constraints[0].orientation.w=1.0
        c1.orientation_constraints[0].absolute_x_axis_tolerance = 0.2 #x axis is pointed up for wrist link
        c1.orientation_constraints[0].absolute_y_axis_tolerance = 0.2
        c1.orientation_constraints[0].absolute_z_axis_tolerance = 6.28
        c1.orientation_constraints[0].weight = 1.0

        while not rospy.is_shutdown():
            result = self.lmove_group.moveToJointPosition(self._left_arm_joints, self.larm_const_stow, 0.05, path_constraints=c1, planning_time=120.0)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def right_arm_constrained_stow(self):
        c1 = Constraints()
        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.stamp = rospy.get_rostime()
        c1.orientation_constraints[0].header.frame_id = "base_link"
        c1.orientation_constraints[0].link_name = "right_ee_link"
        c1.orientation_constraints[0].orientation.w=1.0
        c1.orientation_constraints[0].absolute_x_axis_tolerance = 0.2 #x axis is pointed up for wrist link
        c1.orientation_constraints[0].absolute_y_axis_tolerance = 0.2
        c1.orientation_constraints[0].absolute_z_axis_tolerance = 6.28
        c1.orientation_constraints[0].weight = 1.0

        while not rospy.is_shutdown():
            result = self.rmove_group.moveToJointPosition(self._right_arm_joints, self.rarm_const_stow, 0.05, path_constraints=c1, planning_time=120.0)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
            
    def open_gripper(self):
        self._lgripper.command(self._gripper_open,block=True)
        self._rgripper.command(self._gripper_open,block=True)
        
    def close_gripper(self):
        self._lgripper.command(self._gripper_closed,block=True)
        self._rgripper.command(self._gripper_closed,block=True) 
        
def convert_dict_to_pose2d(loc):
    
    tmp=Pose2D(loc["x"],loc["y"],loc["theta"])
    return tmp
        

if __name__ == "__main__":
    
    # Create a node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("lx_pick_place_demo")
    
    """
    Get all the demo locations from the parameter file defined by the user
    """
    table_height = rospy.get_param("~table_height",0.74)
    
    is_sim = rospy.get_param("~sim",False)
    
    if (is_sim):
        rospy.wait_for_message('/sim_initialized',Bool)

    
    # Setup clients
    move_base = MoveBaseActionClient(sim=is_sim,frame="odom")
    head_action = PointHeadClient()
    grasping_client = GraspingClient(sim=is_sim)
    grasping_client.clearScene()
    movo_torsor = TorsoActionClient()
    # my_trans = my_transformer()
    
    

    # def r_to_q(r_list, q_org):
    #     rx, ry, rz = r_list
    #     q_rot = quaternion_from_euler(rx, ry, rz)
    #     q_new = quaternion_multiply(q_rot, q_org)
    #     return q_new

    # def get_ee_from_finger(pose_finger):
    #     pass

 

    # def prepare():
    # # set the robot pos to the planning pose
    #     # raw_input("========== Press Enter to goto_tuck ===========")
    #     # grasping_client.goto_tuck()   
    #     grasping_client.close_gripper()
    #     raw_input("========== Press Enter to goto_plan_grasp ===========")
    #     grasping_client.goto_plan_grasp()
    #     # grasping_client.goto_plan_grasp_lx()
    #     grasping_client.open_gripper()


    # def grab_casing():
    #     raw_input("========== Press Enter to move-in grippers =====")
        
    #     rg_pose = PoseStamped()
    #     rg_pose.header.frame_id = 'right_ee_link'
    #     r_pose = grasping_client._listener.transformPose('/base_link', rg_pose)
    #     r_pose.pose.position.y += 0.09
    #     grasping_client.rmove_group.moveToPose(r_pose, 'right_ee_link', 0.005, max_velocity_scaling_factor=0.1)
    #     # rospy.sleep(1.0)
    #     grasping_client._rgripper.command(0.07,block=True)

    #     lg_pose = PoseStamped()
    #     lg_pose.header.frame_id = 'left_ee_link'
    #     l_pose = grasping_client._listener.transformPose('/base_link', lg_pose)
    #     l_pose.pose.position.y -= 0.09
    #     grasping_client.lmove_group.moveToPose(l_pose, 'left_ee_link', 0.005, max_velocity_scaling_factor=0.1)
    #     # rospy.sleep(1.0)
    #     grasping_client._lgripper.command(0.07,block=True)

    
    # def lift():
    #     # raw_input("========== Press Enter to vibrate gripper =====")
    #     lg_pose = PoseStamped()
    #     lg_pose.header.frame_id = 'left_ee_link'
    #     l_pose = grasping_client._listener.transformPose('/base_link', lg_pose)

    #     rg_pose = PoseStamped()
    #     rg_pose.header.frame_id = 'right_ee_link'
    #     # r_pose = grasping_client._listener.transformPose('/base_link', rg_pose)

    #     #left
    #     # pose0 = l_pose
    #     # l_pose.pose.position.x += 0.015
    #     # pose1 = l_pose

    #     # l_pose.pose.position.y -= 0.015
    #     # pose2 = l_pose

    #     # pose0 = lg_pose
    #     # lg_pose.pose.position.x += 0.015
    #     # pose1 = lg_pose

    #     # lg_pose.pose.position.y -= 0.015
    #     # pose2 = lg_pose


    #     raw_input("========== Press Enter to rotate gripper =====")
        

    #     # lg_pose.pose.position.z += 0.2
    #     # rg_pose.pose.position.z -= 0.2
    #     # grasping_client.lmove_group.moveToPose(lg_pose, 'left_ee_link', 0.05, wait=False, max_velocity_scaling_factor=0.2)
    #     # grasping_client.rmove_group.moveToPose(rg_pose, 'right_ee_link', 0.05, max_velocity_scaling_factor=0.2)
    #     '''
    #     movo_torsor.clear()
    #     temp_torso = rospy.wait_for_message("/movo/linear_actuator/joint_states", JointState)
    #     current_torso_pos = list(temp_torso.position)
    #     movo_torsor.add_point(current_torso_pos, 0.0)
    #     movo_torsor.add_point([0.35], 6)
    #     movo_torsor.start()
    #     # movo_torsor.wait(14.0)
    #     # print "Movo Completing Torso Motion"
    #     '''
    #     # grasping_client.lmove_group.moveToPose(lg_pose, 'left_ee_link', 0.01)
     
    #     l_q = r_to_q([math.pi/2, 0, 0], [0,0,0,1])

    #     lg_pose.pose.orientation.x = l_q[0]
    #     lg_pose.pose.orientation.y = l_q[1]
    #     lg_pose.pose.orientation.z = l_q[2]
    #     lg_pose.pose.orientation.w = l_q[3]
    #     rospy.sleep(1.0)
    #     grasping_client.lmove_group.moveToPose(lg_pose, 'left_ee_link', 0.01)
    #     # lg_pose.pose.position.x -= 0.01
    #     # # rospy.sleep(1.0)
    #     # grasping_client.lmove_group.moveToPose(lg_pose, 'left_ee_link', 0.01)
    #     # lg_pose.pose.position.y += 0.01
    #     # # rospy.sleep(1.0)
    #     # grasping_client.lmove_group.moveToPose(lg_pose, 'left_ee_link', 0.01)
    #     # lg_pose.pose.position.y -= 0.01
    #     # # rospy.sleep(1.0)
    #     # grasping_client.lmove_group.moveToPose(lg_pose, 'left_ee_link', 0.01)
    #     # rospy.sleep(1.0)

    
    # def shake_joint():
    #     pass


    # prepare()
    # rospy.sleep(2.0)
    # # raw_input("==== Manoeuvre base and camera to proper pose using Joystick and Press Enter")
    # grab_casing()
    # lift()
    
    # torso.add_point([0.30], 5.0)
    # raw_input("========== Press Enter to start torso =====")
    # torso.start()
    
    '''
    mb = tf.TransformerROS(True, rospy.Duration(10.0))
    marker_base = TransformStamped()
    marker_base.header.frame_id = '/base_link'
    marker_base.child_frame_id = '/ar_marker_17'
    
    point1 = grasping_client._listener.lookupTransform('/base_link', '/movo_camera_color_optical_frame', rospy.Time(0))
    rospy.loginfo("========= Frame from base to camera is: ")
    rospy.loginfo(point1)
    point2 = grasping_client._listener.lookupTransform('/movo_camera_color_optical_frame', '/ar_marker_17', rospy.Time(0))
    rospy.loginfo("========= Frame from camera to marker is: ")
    rospy.loginfo(point2)
    tmp = point_comb(point1, point2, 'base_link')

    marker_base.transform.translation.x = tmp.pose.position.x
    marker_base.transform.translation.y = tmp.pose.position.y
    marker_base.transform.translation.z = tmp.pose.position.z
    marker_base.transform.rotation = tmp.pose.orientation
    mb.setTransform(marker_base)    
    look_marker = mb.lookupTransform('/base_link', '/ar_marker_17', rospy.Time(0))
    rospy.loginfo("========= Frame from base to marker is: ")
    rospy.loginfo(look_marker)
    rospy.loginfo("========= Frame from marker to base is: ")
    rospy.loginfo(mb.lookupTransform('/ar_marker_17', '/base_link', rospy.Time(0)))
    '''


    def pp_nut():
        # TO DO: 
        # pick nut
        raw_input("========== Put a nut and Enter to close left gripper ===========")
        grasping_client.close_gripper()


        # place nut:
        # where should finger tip be with reference to '/base_link'
        pose_nutStart_nut = PoseStamped()
        pose_nutStart_nut.header.frame_id = '/nutStart'
        pose_nutStart_ar = my_trans.tf.transformPose('/ar_marker_17', pose_nutStart_nut)

        grasping_client._listener.waitForTransform("/ar_marker_17", "/movo_camera_color_optical_frame", rospy.Time(), rospy.Duration(4.0))
        pose_nutStart_ca = grasping_client._listener.transformPose('/movo_camera_color_optical_frame', pose_nutStart_ar)
        pose_leeStart_ca = pose_nutStart_ca
        pose_leeStart_ca.pose.position.x -= 0.043
        pose_leeStart_ca.pose.position.z -= 0.02

        # pose_leeStart_bs = grasping_client._listener.transformPose('/base_link', pose_leeStart_ca)
        

        rospy.loginfo("======== Target position for nutStart(/ar_marker_17) is:")
        rospy.loginfo(pose_nutStart_ar)
        rospy.loginfo("======== Target position for left_ee(/camera) is:")
        rospy.loginfo(pose_leeStart_ca)
        # rospy.loginfo("======== Target position for left_ee(/base_link) is:")
        # rospy.loginfo(pose_leeStart_bs)

        raw_input("========== Press Enter to move gripper to nutStart ===========")
        grasping_client.lmove_group.moveToPose(pose_leeStart_ca, "left_ee_link")

        '''
        while not rospy.is_shutdown():
            try:
                result = grasping_client.lmove_group.moveToPose(pose_leeStart_ca, "left_ee_link", tolerance = 0.005)
            except: continue

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                break
        '''
                  
        
    
    def rot_nut():
        # pos_lee_bl = grasping_client._listener.lookupTransform('/base_link', '/fr_nut', rospy.Time(0))
        # pos_nut_fn = transformPose('/fr_nut', pos_lee)
        pose_nutEnd_nut = PoseStamped()
        pose_nutEnd_nut.header.frame_id = '/nutStart'
        pose_nutEnd_nut.pose.position.x = 0.05
        qua = r_to_q([math.pi, 0, 0], [0, 0, 0, 1])
        pose_nutEnd_nut.pose.orientation = Quaternion(qua[0], qua[1], qua[2], qua[3])

        pose_nutEnd_ar = my_trans.tf.transformPose('/ar_marker_17', pose_nutEnd_nut)
        pose_nutEnd_ca = grasping_client._listener.transformPose('/movo_camera_color_optical_frame', pose_nutEnd_ar)
        pose_leeEnd_ca = pose_nutEnd_ca
        pose_leeEnd_ca.pose.position.x -= 0.043
        pose_leeEnd_ca.pose.position.z -= 0.02

        # pose_leeStart_bs = grasping_client._listener.transformPose('/base_link', pose_leeStart_ca)
        raw_input("========== Press Enter to start rotate ===========")
        grasping_client.lmove_group.moveToPose(pose_leeEnd_ca, "left_ee_link")

        '''
        while not rospy.is_shutdown():
            try:
                result = grasping_client.lmove_group.moveToPose(pose_leeEnd_ca, "left_ee_link", tolerance = 0.005)
            except: continue

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                break
        '''





        '''
        rotate_nut = PoseStamped()
        rotate_nut.header.frame_id = "fr_nut"
        rotate_nut.pose.position.x = 0.1
        # rotate_nut.pose.position.y = 0
        # rotate_nut.pose.position.z = 0
        q_org = [0, 0, 0, 1]
        q_new = r_to_q([math.pi, 0, 0], q_org)
        rotate_nut.pose.orientation = Quaternion(q_new[0], q_new[1], q_new[2], q_new[3])
        rospy.loginfo("========= Target pose for nut frame is:")
        rospy.loginfo(rotate_nut)

        # rotate_left_ee = PoseStamped()
        # rotate_left_ee.header.frame_id = "left_ee_link"
        rotate_left_ee = tle.transformPose('/left_ee_link', rotate_nut)
        rospy.loginfo("========= Target pose for left e-e is:")
        rospy.loginfo(rotate_left_ee)
        raw_input("========== Press Enter to rotate nut ===========")
        grasping_client.lmove_group.moveToPose(rotate_left_ee, "left_ee_link")
        '''



    def screw():
        curr_pos = grasping_client._listener.lookupTransform('/base_link', '/left_ee_link', rospy.Time(0))
        rospy.loginfo(curr_pos)
        raw_input("========== Put a nut and Enter to close left gripper ===========")
        grasping_client.close_gripper()
        move_rl = PoseStamped()
        move_rl.header.frame_id = "left_ee_link"
        move_rl.pose.position.x = 0.05
        # move_rl.pose.position.y = 0.01
        # move_rl.pose.position.z = -0.5
        # q_org = curr_pos[1]
        q_org = [0, 0, 0, 1]
        q_new = r_to_q([math.pi, 0, 0], q_org)
        move_rl.pose.orientation = Quaternion(q_new[0], q_new[1], q_new[2], q_new[3])

        # grasping_client.lmove_group.set_pose_target(move_rl)
        # grasping_client.lmove_group.go(wait=True)
        # grasping_client.lmove_group.stop()
        # grasping_client.lmove_group.clear_pose_targets()
        raw_input("========== Press Enter to rotate left gripper ===========")
        grasping_client.lmove_group.moveToPose(move_rl, "left_ee_link")
        # grasping_client.lmove_group.moveToJointPosition(grasping_client._left_arm_joints[-1], 1.0)

        '''
        raw_input("========== Press Enter to execute head_action ===========")
        head_action.look_at(1.8, 0, table_height+.1, "base_link")
        
        while not rospy.is_shutdown():
            coords = grasping_client.getPickCoordinates()
            if coords == None:
                rospy.logwarn("Perception failed.")
                continue         
            break

        
        raw_input("========== Press Enter to move base to table ===========")
        rospy.loginfo("Moving to table...")
        move_base.goto(coords)
        
        # Point the head at the stuff we want to pick
        raw_input("========== Press Enter to move head to pick area ===========")
        head_action.look_at(0.9, 0.1, table_height+.1, "base_link")

        rospy.loginfo("========= Testing is complete... ==========")
        '''
    
    # rospy.sleep(1.0)
    # screw()
    # pp_nut()
    # pp_bolt()
    # rospy.sleep(1.0)
    # rot_nut()

    '''




























    
    # find stuff to pick
    while not rospy.is_shutdown():
        rospy.loginfo("Picking object...")
        grasping_client.updateScene(0)
        nut, grasps = grasping_client.getGraspableNut()
        if nut == None:
            rospy.logwarn("Perception failed.")
            continue

        # Pick the nut
        if grasping_client.pick(nut, grasps,0):
            break
        rospy.logwarn("Grasping failed.")

    # Goto grasping position
    grasping_client.goto_plan_grasp()
    
    # Point the head at the stuff we want to pick
    head_action.look_at(0.9, -0.1, table_height+.1, "base_link")

    while not rospy.is_shutdown():
        rospy.loginfo("Picking object...")
        grasping_client.updateScene(1)
        bolt, grasps = grasping_client.getGraspableBolt()
        if bolt == None:
            rospy.logwarn("Perception failed.")
            continue

        # Pick the bolt
        if grasping_client.pick(bolt, grasps,1):
            break
        rospy.logwarn("Grasping failed.")
        
    
    # Goto grasping position
    grasping_client.goto_plan_grasp()
    
    ## Place
    # Point the head at the stuff we want to place
    head_action.look_at(0.9, 0.1, table_height+.1, "base_link")
    
    # Place the nut
    while not rospy.is_shutdown():
        rospy.loginfo("Placing object...")
        pose = PoseStamped()
        pose.pose = nut.primitive_poses[0]
        pose.pose.position.z += 0.01
        pose.header.frame_id = nut.header.frame_id
        if grasping_client.place(nut, pose, gripper=0):
            break
        rospy.logwarn("Placing failed.")
        
    grasping_client.goto_plan_grasp()


    # Point the head at the stuff we want to pick
    head_action.look_at(0.9, -0.1, table_height+.1, "base_link")
    
    # Place the bolt
    while not rospy.is_shutdown():
        rospy.loginfo("Placing object...")
        pose = PoseStamped()
        pose.pose = bolt.primitive_poses[0]
        pose.pose.position.z += 0.01
        pose.header.frame_id = bolt.header.frame_id
        if grasping_client.place(bolt, pose, gripper=1):
            break
        rospy.logwarn("Placing failed.")
        
    grasping_client.goto_plan_grasp()
    ##

    
    # Demo finished return to tuck
    grasping_client.close_gripper()
    grasping_client.goto_tuck()

    rospy.loginfo("Moving to start...")
    target = Pose2D(x=0.0,y=0.0,theta=0.0)
    move_base.goto(target)
    
    rospy.loginfo("Demo is complete...")
'''