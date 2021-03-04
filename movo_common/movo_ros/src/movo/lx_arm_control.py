from movo_msgs.msg import JacoCartesianVelocityCmd

import rospy
import sys
import time

import movo_arc_lib.msg as arcmsg
import actionlib

class arm_control(object):
    def __init__(self):

        rospy.init_node('lx_arm_control')       

        self.arm_pub = [0]*2
        # self.init_arm_pub =[0]*2

        self.insert_speed = 0.05
        self.cartesianforce_left = [0, 0, 0, 0, 0, 0]
        self.cartesianforce_right = [0, 0, 0, 0, 0, 0]       

        # 0: left, 1: right
        self.arm_pub[0] = rospy.Publisher('/movo/left_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
        self.arm_pub[1] = rospy.Publisher('/movo/right_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)

        self.default_pose_pickready = [0.642, -0.936, 0.767, -1.46, -0.928, 1.68, -3.14/4,\
            -0.642, 0.936, -0.767, 1.46, 0.928, -1.68, 3.14/4,\
                0.46, 0, 0]
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

        # move joint safe
        name="L0_upper_jp_move_safe"
        action = arcmsg.upper_jp_movo_safeAction
        self.upper_move_client = actionlib.SimpleActionClient(name,action)
        self.upper_move_client.wait_for_server()

        # move joint safe
        name="L0_dual_task_move_safe_relate"
        action = arcmsg.dual_task_move_safe_relateAction
        self.dual_rela_client= actionlib.SimpleActionClient(name,action)
        self.dual_rela_client.wait_for_server()


    def reset(self):
        args = self.default_pose_pickready
        goal = arcmsg.upper_jp_movo_safeGoal()
        goal.jp_left = args[7:14]
        goal.jp_right = args[0:7]
        goal.jp_head = args[15:17]
        goal.jp_linear = args[14:15][0]
        goal.duration = 300
        goal.l_max_force=[20 for i in range(6)]
        goal.r_max_force=[20 for i in range(6)]
        self.upper_move_client.send_goal(goal)
        self.upper_move_client.wait_for_result()
        # print "Goal Sent and Finished!"
        return self.upper_move_client.get_result() 

    def move_relate(self,arm,action):
        # action = [-1, 1]
        action=list(map(lambda x:x/100,action))
        lmove=[0 for i in range(3)]
        rmove=[0 for i in range(3)]
        if(arm=="left"):
            lmove[0]=action[0]
            lmove[2]=action[1]
        elif(arm=="right"):
            rmove[0]=action[0]
            rmove[2]=action[1]
        time=1
        mygoal = arcmsg.dual_task_move_safe_relateGoal()
        mygoal.pos_r=rmove
        mygoal.pos_l=lmove
        mygoal.time=time
        mygoal.r_max_force=[10 for i in range(6)]
        mygoal.l_max_force=[10 for i in range(6)]

        # send a goal
        self.dual_rela_client.send_goal(mygoal)
        self.dual_rela_client.wait_for_result()
        print("Move Finished")
    def move(self, arm, le_ri=0, up_dn=0):
        raise NotImplemented("try move_relate")
        '''
        le_ri: [-1, 1]
        up_dn: [-1, 1]
        '''
        assert arm in ['left', 'right']
        vel_x = 0.15*le_ri
        vel_y = -0.15*up_dn
        vel_z = self.insert_speed

        arm_cmd = JacoCartesianVelocityCmd()
        arm_cmd.header.stamp=rospy.get_rostime()
        arm_cmd.header.frame_id=''
        arm_cmd.x = vel_x
        arm_cmd.y = vel_y
        arm_cmd.z = vel_z

        r = rospy.Rate(1000)
        if arm == 'left':
            timeout = time.time()+0.05
            while not rospy.is_shutdown():
                self.arm_pub[0].publish(arm_cmd)
                if time.time()>timeout:
                    break
                r.sleep()
                print('run')

        if arm == 'right':
            timeout = time.time()+0.05
            while not rospy.is_shutdown():
                self.arm_pub[1].publish(arm_cmd)
                if time.time()>timeout:
                    break
                r.sleep()
                print('run')

    def random_move(self):
        pass
    
    def _subscribe_force_callback_left(self, data):
        # fx = data.x
        # fy = data.y
        # fz = data.z
        # fr = data.theta_x
        # fp = data.theta_y
        # fa = data.theta_z
        # print data
        self.cartesianforce_left[0] = data.x
        self.cartesianforce_left[1] = data.y
        self.cartesianforce_left[2] = data.z
        self.cartesianforce_left[3] = data.theta_x
        self.cartesianforce_left[4] = data.theta_y
        self.cartesianforce_left[5] = data.theta_z
        # self.cartesianforce_left = [fx, fy, fz, fr, fp, fa]

    def _subscribe_force_callback_right(self, data):
        fx = data.x
        fy = data.y
        fz = data.z
        fr = data.theta_x
        fp = data.theta_y
        fa = data.theta_z
        # print data
        self.cartesianforce_right = [fx, fy, fz, fr, fp, fa]

    def get_force_z(self, arm):
        assert arm in ["left", "right"]

        rospy.Subscriber("/movo/left_arm/cartesianforce", JacoCartesianVelocityCmd,self._subscribe_force_callback_left)
        rospy.Subscriber("/movo/right_arm/cartesianforce", JacoCartesianVelocityCmd,self._subscribe_force_callback_right)

        if arm == "left":
            return self.cartesianforce_left[2]
        else:
            return self.cartesianforce_right[2]

if __name__ == '__main__':
    arm = arm_control()
    ARM = "left"

    arm.reset()
    while(True):
        arm.move_relate("right", [0.5, 0])
        arm.move_relate("right", [0.2, 0])


    # arm.move(ARM,0,0.5)

    #
    # while True:
    #     # print("===== Force detection is: {} N".format(arm.cartesianforce_left))
    #     print("===== Force detection is: {} N".format(arm.get_force_z(ARM)))
    #     # if KeyboardInterrupt:
    #     #     break

    # raw_input("===== Press Enter to reset =====")
    # arm.reset()
   
    '''
    step=0
    while True:
        step+=1
        img=get_img_from_cam()
        action = algorithm(img) # action=[0.5,0.5]
        arm.move(ARM,action[0],action[1])

        force_detect = arm.get_force_z(ARM)
        if(force_detect>5):
            arm.reset()
            raw_input("===== Press Enter to Restart =====")
        if(step>100):
            arm.reset()
    '''

    rospy.spin()


        
