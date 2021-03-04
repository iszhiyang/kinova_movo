import rospy, sys
import thread, copy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy

class MoveAttachedObjectDemo:
    def __init__(self):
        # Initialize API for movo_group
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize Node of ROS
        rospy.init_node('zhiyang_attached_camera_demo')
        
        # Define Scene Object
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                                
        # Initialize arm group from moveit group to control manipulator
        rarm = MoveGroupCommander('right_arm')
        larm = MoveGroupCommander('left_arm')
        
        # Get end effector link name
        r_end_effector_link = rarm.get_end_effector_link()
        l_end_effector_link = larm.get_end_effector_link()
        
        # Set tolerance of position(m) and orentation(rad)
        rarm.set_goal_position_tolerance(0.01)
        rarm.set_goal_orientation_tolerance(0.05)
        larm.set_goal_position_tolerance(0.01)
        larm.set_goal_orientation_tolerance(0.05) 

        # Allow replanning
        rarm.allow_replanning(True)
        rarm.set_planning_time(10)
        larm.allow_replanning(True)
        larm.set_planning_time(10)
        # Let arm go back to home position
        # arm.set_named_target('home')
        # arm.go()
        
        # Remove other attached object in before Scene run
        # scene.remove_attached_object(end_effector_link, 'tool')
        # scene.remove_world_object('table') 
        # scene.remove_world_object('target')

        # Set height of table
        table_ground = 0.6
        
        # Set 3d size of table and tool
        table_size = [0.3, 0.7, 0.01]
        tool_size = [0.15, 0.15, 0.06]
        
        # Set orentation and position of tool
        p1 = PoseStamped()
        p1.header.frame_id = r_end_effector_link
        
        # p.pose.position.x = tool_size[0] / 2.0 - 0.025
        # p.pose.position.y = -0.015
        # p.pose.position.z = 0.0
        p1.pose.position.x = -0.07
        p1.pose.position.y = 0.0
        p1.pose.position.z = 0.08
        p1.pose.orientation.x = 0
        p1.pose.orientation.y = 0
        p1.pose.orientation.z = 0
        p1.pose.orientation.w = 1

        p2 = PoseStamped()
        p2.header.frame_id = l_end_effector_link
        
        p2.pose.position.x = -0.07
        p2.pose.position.y = 0.0
        p2.pose.position.z = 0.08
        p2.pose.orientation.x = 0
        p2.pose.orientation.y = 0
        p2.pose.orientation.z = 0
        p2.pose.orientation.w = 1
        
        # Make tool attact to end effector
        scene.attach_box(r_end_effector_link, 'Camera1', p1, tool_size)
        scene.attach_box(l_end_effector_link, 'Camera2', p2, tool_size)

        # Add table to Scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.position.x = 0.75
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box('table', table_pose, table_size)
        
        rospy.sleep(2)  

        # Update current state (position and orentation)
        rarm.set_start_state_to_current_state()
        larm.set_start_state_to_current_state()

        # Set target (rad)
        joint_positions = [0.827228546495185, 0.29496592875743577, 0.29496592875743577, 1.1185644936946095, -0.7987583317769674, -0.18950024740190782, 0.11752152218233858]
        rarm.set_joint_value_target(joint_positions)
        larm.set_joint_value_target(joint_positions)
                 
        # Let arm go
        rarm.go()
        larm.go()
        rospy.sleep(1)
        
        # Let arm go back to initial state
        # arm.set_named_target('home')
        # arm.go()

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveAttachedObjectDemo()