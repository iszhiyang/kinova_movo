#!/usr/bin/env python	
import copy	
import actionlib	
import rospy	
import sys	
import tf	

from moveit_msgs.msg import Constraints, OrientationConstraint, Grasp	

from moveit_python import (MoveGroupInterface,	
                           PlanningSceneInterface,	
                           PickPlaceInterface)	
from moveit_python.geometry import rotate_pose_msg_by_euler_angles	

from movo_action_clients.gripper_action_client import GripperActionClient	
from movo_action_clients.move_base_action_client import MoveBaseActionClient	
from movo_action_clients.torso_action_client import TorsoActionClient	

from control_msgs.msg import PointHeadAction, PointHeadGoal	
from grasp_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal, GraspPlanningAction, GraspPlanningGoal, Object
from shape_msgs.msg import SolidPrimitive as sp	
from geometry_msgs.msg import Pose2D,PoseStamped,Pose
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes	
from std_msgs.msg import Bool	

def pause():
    rospy.loginfo("Pausing")
    raw_input()
    rospy.loginfo("Starting")

# Tools for grasping	
class GraspingClient(object):	

    def __init__(self,sim=False):
        # Define planning groups
        self.scene = PlanningSceneInterface("base_link")	
        self.dof = rospy.get_param('~jaco_dof')	
        self.move_group = MoveGroupInterface("upper_body", "base_link")	
        self.rmove_group = MoveGroupInterface("right_arm", "base_link")	

        planner_id = "RRTConnectkConfigDefault"	
        self.move_group.setPlannerId(planner_id)	
        self.rmove_group.setPlannerId(planner_id)	

        self.objects_heights = [0.122, 0.240]	
        self.objects_heights_tolerances = [0.02, 0.03]	

        # Define joints and positions for 6 and 7 DOF
        if "6dof" == self.dof:	
            self._upper_body_joints = ["right_elbow_joint",	
                            "right_shoulder_lift_joint",	
                            "right_shoulder_pan_joint",	
                            "right_wrist_1_joint",	
                            "right_wrist_2_joint",	
                            "right_wrist_3_joint",	
                            "left_elbow_joint",	
                            "left_shoulder_lift_joint",	
                            "left_shoulder_pan_joint",	
                            "left_wrist_1_joint",	
                            "left_wrist_2_joint",	
                            "left_wrist_3_joint",	
                            "linear_joint",	
                            "pan_joint",	
                            "tilt_joint"]	
            self._right_arm_joints = ["right_elbow_joint",	
                            "right_shoulder_lift_joint",	
                            "right_shoulder_pan_joint",	
                            "right_wrist_1_joint",	
                            "right_wrist_2_joint",	
                            "right_wrist_3_joint"]	
            self._left_arm_joints = ["left_elbow_joint",	
                            "left_shoulder_lift_joint",	
                            "left_shoulder_pan_joint",	
                            "left_wrist_1_joint",	
                            "left_wrist_2_joint",	
                            "left_wrist_3_joint"]	
            self.tucked = [-2.8,-1.48,-1.48,0,0,1.571,2.8,1.48,1.48,0,0,-1.571,0.0371,0.0,0.0]	
            self.constrained_stow = [2.28,2.00,-2.56,-0.09,0.15,1.082,-2.28,-2.17,2.56,0.09,-0.15,2.06,0.42,0.0,0.0]		
            self.rarm_const_stow = [2.28,2.00,-2.56,-0.09,0.15,1.06]	
            #self.rarm_const_stow = [-1.51, -0.22, -2.00, -2.04, 1.42, 1.32]

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
            self.constrained_stow =[-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, 1.0, 2.6, -2.0, 0.0, -2.0, 0.0, 0.0, -1.0, 0.42, 0, 0]		
            self.rarm_const_stow = [-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, -1.0]

        else:	
            rospy.logerr("DoF needs to be set 6 or 7, aborting demo")	
            return;	

        # Define the MoveIt pickplace pipeline
        self.pickplace = None
        self.pickplace = PickPlaceInterface("left_side", "left_gripper", verbose=True)	
        self.pickplace.planner_id = planner_id	
        self.pick_result = None		
        self.place_result = None	

        self._listener = tf.TransformListener()	
	
        self._lgripper = GripperActionClient('left')	

        find_grasp_planning_topic = "grasp_planner/plan"	
        rospy.loginfo("Waiting for %s..." % find_grasp_planning_topic)	
        self.find_grasp_planning_client = actionlib.SimpleActionClient(find_grasp_planning_topic, GraspPlanningAction)	
        self.find_grasp_planning_client.wait_for_server()	
        rospy.loginfo("...connected")	

        self.scene.clear()	

        # This is a simulation so need to adjust gripper parameters	
        if sim:	
            self._gripper_closed = 0.96	
            self._gripper_open = 0.00	
        else:	
            self._gripper_closed = 0.01	
            self._gripper_open = 0.165	

    def __del__(self):	
        self.scene.clear()	

    def add_objects_to_keep(self,obj):	
        self._objs_to_keep.append(obj)	

    def clearScene(self):	
        self.scene.clear()	

    def calculateGraspForObject(self, object_to_grasp, gripper):	
        goal = GraspPlanningGoal()	
        goal.object = object_to_grasp	
        goal.gripper = gripper	
        self.find_grasp_planning_client.send_goal(goal)	
        self.find_grasp_planning_client.wait_for_result(rospy.Duration(5.0))	

        return self.find_grasp_planning_client.get_result().grasps #moveit_msgs/Grasp[]	

    def pick(self, block, grasps):	
        success, pick_result = self.pickplace.pick_with_retry(block.name,	
                                                              grasps,	
                                                              retries=1,	
                                                              scene=self.scene)	
        self.pick_result = pick_result	
        return success	

    def place(self, block, pose_stamped):	
        places = list()	
        l = PlaceLocation()	
        l.place_pose.pose = pose_stamped.pose	
        l.place_pose.header.frame_id = pose_stamped.header.frame_id	

        # copy the posture, approach and retreat from the grasp used	
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture	
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach	
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat	
        places.append(copy.deepcopy(l))	
        # create another several places, rotate each by 360/m degrees in yaw direction	
        m = 16 # number of possible place poses	
        pi = 3.141592653589	
        for i in range(0, m-1):	
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)	
            places.append(copy.deepcopy(l))	

        success, place_result = self.pickplace.place_with_retry(block.name,	
                                                                         places,	
                                                                         retries=1,	
                                                                         scene=self.scene)	
        self.place_result = place_result	
        return success	

    def goto_tuck(self):	
        # remove previous objects	
        while not rospy.is_shutdown():	
            result = self.move_group.moveToJointPosition(self._upper_body_joints, self.tucked, 0.05)	
            if result.error_code.val == MoveItErrorCodes.SUCCESS:	
                return	

    def goto_plan_grasp(self):	
        while not rospy.is_shutdown():	
            result = self.move_group.moveToJointPosition(self._upper_body_joints, self.constrained_stow, 0.05)	
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
        # c1 = Constraints()	
        # c1.orientation_constraints.append(OrientationConstraint())	
        # c1.orientation_constraints[0].header.stamp = rospy.get_rostime()	
        # c1.orientation_constraints[0].header.frame_id = "base_link"	
        # c1.orientation_constraints[0].link_name = "right_ee_link"	
        # c1.orientation_constraints[0].orientation.w=1.0	
        # c1.orientation_constraints[0].absolute_x_axis_tolerance = 0.2 #x axis is pointed up for wrist link	
        # c1.orientation_constraints[0].absolute_y_axis_tolerance = 0.2	
        # c1.orientation_constraints[0].absolute_z_axis_tolerance = 6.28	
        # c1.orientation_constraints[0].weight = 1.0	

        while not rospy.is_shutdown():	
            result = self.rmove_group.moveToJointPosition(self._right_arm_joints, self.rarm_const_stow, 0.05, planning_time=120.0)	
            if result.error_code.val == MoveItErrorCodes.SUCCESS:	
                return	

    def open_gripper(self):	
        self._lgripper.command(self._gripper_open,block=True)	

    def close_gripper(self):	
        self._lgripper.command(self._gripper_closed,block=True) 	

def convert_dict_to_pose2d(loc):	

    tmp=Pose2D(loc["x"],loc["y"],loc["theta"])	
    return tmp	


if __name__ == "__main__":	

    # Create a node	
    rospy.init_node("pick_place_demo")	

    # If simulation, wait for initilization
    is_sim = rospy.get_param("~sim",False)	
    if (is_sim):	
        rospy.wait_for_message('/sim_initialized',Bool)	

    # Setup client
    grasping_client = GraspingClient(sim=is_sim)	
    grasping_client.clearScene()	

    # Set the robot initial position, ready to pick
    rospy.loginfo("Going to initial position...")
    grasping_client.goto_plan_grasp()
    grasping_client.open_gripper()
    rospy.loginfo("Reached initial position")

    # Define support surface and add it to planning scene
    surface = Object()
    surface.name = "MyTable"
    surface.header.frame_id = "base_link"
    x_width = 1.0 #m
    y_width = 2.0 #m
    z_width = 0.75 #m
    primitive = sp()
    primitive.type = primitive.BOX
    primitive.dimensions = [x_width, y_width, z_width]
    surface.primitives = [primitive]
    object_pose = Pose()
    object_pose.position.x = 1.0
    object_pose.position.y = 0.0
    object_pose.position.z = 0.375
    object_pose.orientation.x = object_pose.orientation.y = object_pose.orientation.z = 0.0
    object_pose.orientation.w = 1.0
    surface.primitive_poses = [object_pose]	
    grasping_client.scene.addSolidPrimitive(surface.name,	
                                    surface.primitives[0],	
                                    surface.primitive_poses[0],	
                                    wait = True)

    # Define object to pick and add to planning scene and add object to planning scene
    object_to_pick = Object()
    object_to_pick.name = "MyObjectToGrasp"
    object_to_pick.header.frame_id = "base_link"
    height = 0.15 #m
    radius = 0.035 #m
    primitive = sp()
    primitive.type = primitive.CYLINDER
    primitive.dimensions = [height, radius]
    object_to_pick.primitives = [primitive]
    object_pose = Pose()
    object_pose.position.x = 0.85
    object_pose.position.y = 0.1
    object_pose.position.z = 0.875
    object_pose.orientation.x = object_pose.orientation.y = object_pose.orientation.z = 0.0
    object_pose.orientation.w = 1.0
    object_to_pick.primitive_poses = [object_pose]
    grasping_client.scene.addSolidPrimitive(object_to_pick.name,	
                                    object_to_pick.primitives[0],	
                                    object_to_pick.primitive_poses[0],	
                                    wait = True)

    # Sync the planing scene
    grasping_client.scene.waitForSync()	

    print grasping_client.scene.getKnownCollisionObjects()

    # Calculate the possible grasps
    pick_has_succeeded = False
    tries = 0
    max_tries = 5
    gripperwanted = 0 #left
    found_grasp = grasping_client.calculateGraspForObject(object_to_pick, gripperwanted)
    print "Found %d graps"%len(found_grasp)
    
    pause()

    # Plan the pick	
    while not rospy.is_shutdown() and tries < max_tries:	
        rospy.loginfo("Picking object...")	

        # Pick it
        if grasping_client.pick(object_to_pick, found_grasp):	
            pick_has_succeeded = True	
            break	
        rospy.logwarn("Grasping failed at try %d."%tries)	
        tries += 1	

    # Go back to an intermediate position
    # grasping_client.goto_plan_grasp()
    
    pause()

    # Plan the place
    # If the pick failed we can't place it back	
    if not pick_has_succeeded:
        rospy.loginfo("Can't place object because it wasn't picked in the first place.")	
    else:
        # Place the object
        tries = 0
        while not rospy.is_shutdown() and tries < max_tries:	
            rospy.loginfo("Placing object back in other position...")	
            pose = PoseStamped()	
            pose.pose = object_to_pick.primitive_poses[0]	
            pose.pose.position.x += 0.2	
            pose.pose.position.y += 0.2	
            pose.pose.position.z += 0.01	
            pose.header.frame_id = object_to_pick.header.frame_id	
            if grasping_client.place(object_to_pick, pose):	
                break	
            rospy.logwarn("Placing failed.")
            tries += 1

            grasping_client.right_arm_constrained_stow()

    # Demo finished return to tuck	
    grasping_client.close_gripper()	
    # grasping_client.goto_tuck()	

    rospy.loginfo("Demo is complete.")
