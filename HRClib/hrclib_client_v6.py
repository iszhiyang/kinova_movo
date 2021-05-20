# Python3 Interfaces for ARC actions libraries
# It connects to py2 servers, with py3 clients
# Email: yiwen.chen@u.nus.edu
# from archieved import movo_arc_lib as arcmsg
import movo_arc_lib.msg as arcmsg
import rospy
import actionlib
# import movo_arc_lib.msg as arcmsg
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatus
import math
import time
#mssage

from geometry_msgs.msg import PoseStamped
from movo_msgs.msg import JacoCartesianVelocityCmd

from movo_action_clients.gripper_action_client import GripperActionClient

# human robot collaboration library
# HRClib


# Icarus architecture
def motion(cmd, args, goal, eval, type):
    if not isinstance(args, tuple):
        args = (args,)
    while True:
        cmd(*args)
        if type == 1:
            dist = sum([(v - goal[i]) ** 2 for i, v in enumerate(eval())]) ** 0.5
            if dist < 0.01:
                break
        elif type == 2:
            if eval() == goal:
                break
        elif type == 3:
            if ((eval() - goal) ** 2) ** 0.5 < 0.001:
                break
        elif type == 4:
            res = eval()
            val = [res == goal[i] for i in range(len(goal))]
            if sum(val) > 0:
                break
class GlobalVariables(object):
    def __init__(self):
        # self.default_pose_tucked = [-1.6, -1.5, 0.4, -2.7, 0.0, 0.5, -1.7,
        #                             1.6, 1.5, -0.4, 2.7, 0.0, -0.5, 1.7,
        #                             0.04, 0, 0]

        self.default_pose_tucked = [-1.595, -1.5, 0.40, -2.612, 0.0, 0.496, -1.69,
                                    1.595, 1.5, -0.4, 2.612, 0.0, -0.496, 1.69,
                                    0.14, 0, -0.6]
        self.default_pose_wings =[-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, 1.0,
                                  2.6, -2.0, 0.0, -2.0, 0.0, 0.0, -1.0,
                                  0.42, 0, 0]  # Preparing Grasping
        self.default_pose_hug= [-0.23, -0.71, -1.02, -1.0, 0.9, 1.89, -2.41,
                                0.44, 0.71, 1.12, 1.21, -1.02, -1.84,2.61, 0.20, 0, 0] # preparing for hold two hands
        self.default_pose_pickready = [0.642, -0.936, 0.767, -1.46,-0.928, 1.68, 2.39,#-1.6, -1.5, 0.4, -2.7, 0.0, 0.5, -1.7,
                                       -0.642, 0.936, -0.767, 1.46,0.928, -1.68, -2.39,
                                      0.46, 0, 0]
        self.default_pose_pickready_lookdown = [0.642, -0.936, 0.767, -1.46, -0.928, 1.68, 2.39,
                                       # -1.6, -1.5, 0.4, -2.7, 0.0, 0.5, -1.7,
                                       -0.642, 0.936, -0.767, 1.46, 0.928, -1.68, -2.39,
                                       0.46, 0, -0.6]
        self.default_pose_horizon_rot_before_insert_ready = \
            [0.642, -0.936, 0.767, -1.46,-0.928, 1.68, 2.39,-1.109,1.122,1.343,1.6119,-0.560,-1.846,2.996,
                                              0.46, 0, -0.6]
        self.default_pose_bolt_pick_ready_pose = [0.477, -0.925, -0.55, -1.550, 1.166, 1.749, 0.704,
           -0.642, 0.936, -0.767, 1.46,0.928, -1.68, -2.39,
           0.46, 0, -0.6]
        self.default_pose_ready_using_screw_driver = [0.642, -0.936, 0.767, -1.46,-0.928, 1.68, 2.39,
                   -0.477, +0.925, +0.55, +1.550,-1.166,-1.749,-0.704,
                   0.46, 0, -0.6]


        self.default_pose_ready_insert_and_screw = {
            'h':(-0.001517469179816544, -0.5982711315155029),
            'lin':0.46091189980506897,
            "l":(0.0148455251805375, 1.0166356471590665, 0.6636496236356031, 1.304654122206155, -0.35251198084942814,
             -1.3975923189794133, -0.7291213085063291),
            "r":(1.090617596451195, -1.194887676820795, -0.11256159107458785, -1.316655125026379, 0.5104717216901622, 1.5209456796141394, 2.7846955116894403)
        }


        # self.default_pose_screw_rightarm_observe = {
        #     'h': (0.001755023142322898, -0.6001875400543213),
        #     'lin': 0.4609679579734802,
        #     'l': (
        #         0.0038434740852024696, 1.0431375621147718, 0.6896942738321421, 1.1683924197678146, -0.32924767083093087,
        #         -1.5484046015075648, -0.7592577059806938),
        #     'r': (
        #         0.6128764574532433, -0.8213907855298692, -0.6226357444258932, -1.5857084310026863, 0.6403175361673652,
        #         1.107163416829694, -2.619071925600757)}

        # self.default_pose_screw_rightarm_observe ={'h': (2.7164242055732757e-05, -0.598494291305542), 'lin': 0.4602239727973938, 'l': (-0.04788406500211284, 0.8859897525113762, 0.6704758381416944, 1.0332433860329095, -0.3700395422742715, -1.5152507759391138, -0.8287116156294152), 'r': (0.6064678266060763, -0.8965656978322989, -0.682343682907415, -1.4767940640080972, 0.5806468819411048, 1.285365911695147, -2.581031332590067)}
        # self.default_pose_screw_rightarm_observe ={'h': (7.456851108145202e-06, -0.598491907119751), 'lin': 0.45954349637031555, 'l': (-0.13673563692145496, 0.9113260018579541, 0.8510354968879774, 1.2026087134531527, -0.3103091333710517, -1.3140966261671398, -0.8170074886917078), 'r': (0.24970810381797293, -0.9000666894013171, -0.409429692167361, -1.6708153362744576, 0.6213650171117617, 1.0436973585759066, -2.4768125180714504)}
        # higher slope
        self.default_pose_screw_rightarm_observe ={'h': (0.0015667376574128866, -0.596684455871582), 'lin': 0.45839282870292664, 'l': (-0.13892220036732894, 0.9125020537953331, 0.8590207191004708, 1.2039381634693203, -0.32134660450888575, -1.3119943268026313, -0.8090667412694188), 'r': (0.3378900613319167, -0.8501308210656773, -0.39544959447255756, -1.7052754423575922, 0.7194772032508259, 1.0104664344938978, -2.55757367707643)}
        # higher slope
        # self.default_pose_screw_rightarm_observe ={'h': (0.0030120352748781443, -0.5952045321464539), 'lin': 0.45910343527793884, 'l': (-0.1285867967494898, 0.9087643072054069, 0.8542893471078359, 1.208193894891267, -0.3167132368443646, -1.3074748092747124, -0.8190381490228926), 'r': (0.6501095800214234, -0.8597435010203184, -0.6368457727913106, -1.5541529015624465, 0.6513479499283097, 1.2091808623912503, -2.610791625139484)}
        # lower slope
        # self.default_pose_screw_rightarm_observe ={'h': (7.456851108145202e-06, -0.598491907119751), 'lin': 0.45954349637031555, 'l': (-0.13673563692145496, 0.9113260018579541, 0.8510354968879774, 1.2026087134531527, -0.3103091333710517, -1.3140966261671398, -0.8170074886917078), 'r': (0.24970810381797293, -0.9000666894013171, -0.409429692167361, -1.6708153362744576, 0.6213650171117617, 1.0436973585759066, -2.4768125180714504)}
        # lower slope 2
        # {'h': (0.0015667376574128866, -0.596684455871582), 'lin': 0.45839282870292664, 'l': (-0.13892220036732894, 0.9125020537953331, 0.8590207191004708, 1.2039381634693203, -0.32134660450888575, -1.3119943268026313, -0.8090667412694188), 'r': (0.3378900613319167, -0.8501308210656773, -0.39544959447255756, -1.7052754423575922, 0.7194772032508259, 1.0104664344938978, -2.55757367707643)}
        self.default_insert1={'h': (-0.0015422365395352244, -0.5958293080329895), 'lin': 0.46071183681488037, 'l': (0.01712263506317724, 1.0175831998749165, 0.6643756013487594, 1.3094861616879947, -0.3512773393680675, -1.3939702867388508, -0.7274227443630785), 'r': (1.6284348559513724, -1.0733641741710622, -0.5672234847826694, -1.2331535732597076, 0.19970890218265858, 1.4587454218264666, 2.4094446625045887)}

        self._gripper_closed = 0.00
        self._gripper_open = 0.165

class myclient(object):
    def __init__(self,name,action,fbmsg):
        self._client=actionlib.SimpleActionClient(name,action)
        print("Connecting...",name)
        self._client.wait_for_server()
        print("Connected",name)
        rospy.Subscriber("/" + name + "/feedback", fbmsg,
                         self._subscribe_actionfb_callback)
        self.fbdata=None

    def __getattr__(self, item):
        return getattr(self._client, item)
    def result_published(self):
        res = self._client.get_result()
        # self._client.
        # if(res.success)
        if (type(res) == type(None)):
            return False
        else:
            return True

    def _subscribe_actionfb_callback(self,data):
        self.fbdata=data.feedback

class odyssey_Interface():
    # simple interface name with L0...
    # ICURAS interface name with i_L0...
    def __init__(self):
        self.g=self.gval=GlobalVariables()
        # self.gval


        # rospy.init_node("odyssey_Interface_py3_node")

        self.client_L0_upper_jp_move_safe=myclient(name="L0_upper_jp_move_safe",
                                                   action=arcmsg.upper_jp_movo_safeAction,
                                                   fbmsg=arcmsg.upper_jp_movo_safeFeedback)

        self.client_L0_dual_jp_move_safe_relate=myclient(name="L0_dual_jp_move_safe_relate",
                                                   action=arcmsg.dual_jp_movo_safe_relateAction,
                                                   fbmsg=arcmsg.dual_jp_movo_safe_relateFeedback)

        self.client_L0_dual_task_move_safe_relate=myclient(name="L0_dual_task_move_safe_relate",
                                                           action=arcmsg.dual_task_move_safe_relateAction,
                                                           fbmsg=arcmsg.dual_task_move_safe_relateFeedback)

        self.client_L0_dual_set_gripper=myclient(name="L0_dual_set_gripper",
                                                 action=arcmsg.dual_set_gripperAction,
                                                 fbmsg=arcmsg.dual_set_gripperFeedback)


        self.client_L0_single_task_move_safe=myclient(name="L0_single_task_move_safe",
                                                 action=arcmsg.single_task_move_safeAction,
                                                 fbmsg=arcmsg.single_task_move_safeFeedback)

        self.client_L0_single_set_gripper=myclient(name="L0_single_set_gripper",
                                                   action=arcmsg.single_set_gripperAction,
                                                   fbmsg=arcmsg.single_set_gripperFeedback)

        self._lgripper = GripperActionClient('left')
        self._rgripper = GripperActionClient('right')

        self.subscribe_force("right")
        self.subscribe_force("left")
        self.subscribe_jointstates()
        self._subscribe_eepose()

        # rospy.Subscriber("/joint_states", JointState,self._subscribe_jointstates_callback)
        #
        # self.jp_updated=False
        # r=rospy.Rate(10)
        # while(not (self.jp_updated and True)):
        #     r.sleep()

        self.cartesianforce_right=None
        self.cartesianforce_left=None
        self.jointstates_pos=None
        self.jointstates_vel=None
        self.jointstates_effort=None
        self.right_arm_js_pos = None
        self.left_arm_js_pos = None
        self.right_arm_js_vel = None
        self.left_arm_js_vel = None
        self.head_js_pos=None
        self.head_js_vel=None
        self.linear_js_pos=None
        self.linear_js_vel=None
        self.lpose= None
        self.rpose= None
        # When you add new states, please regist in _states

            # Wait till all states got updated
        while(sum(type(i)==type(None) for i in self._states)):
            print(self._states)
            time.sleep(1)
            # print(self.jointstates_effort)
            print("Wait till all states got first updated")

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
            self.linear_js_vel,
            self.lpose,
            self.rpose
        ]
    def E0_getjp(self):
        return self.jp
    def E0_getjp_arm(self,arm):

        if(arm=="left" or arm=='l'):
            pos=self.left_arm_js_pos
            print(pos)
            return pos
        if(arm=="right" or arm=='r'):
            return self.right_arm_js_pos
        raise NotImplementedError("ARM not defined")
        # return self.jp



    def subscribe_force_callback_right(self, data):
        fx = data.x
        fy = data.y
        fz = data.z
        fr = data.theta_x
        fp = data.theta_y
        fa = data.theta_z
        # print data
        self.cartesianforce_right = [fx, fy, fz, fr, fp, fa]

    def subscribe_force_callback_left(self, data):
        fx = data.x
        fy = data.y
        fz = data.z
        fr = data.theta_x
        fp = data.theta_y
        fa = data.theta_z
        # print data
        self.cartesianforce_left = [fx, fy, fz, fr, fp, fa]
        # print(self.cartesianforce)
    def subscribe_force(self, arm):
        assert arm in ["left", "right"]
        arm_name = arm+'_arm'
        # print "======== I'm subscribe_force =========="
        if(arm=="left"):
            rospy.Subscriber("/movo/{}/cartesianforce".format(arm_name), JacoCartesianVelocityCmd,
                             self.subscribe_force_callback_left)
        elif(arm=="right"):
            rospy.Subscriber("/movo/{}/cartesianforce".format(arm_name), JacoCartesianVelocityCmd,
                             self.subscribe_force_callback_right)

    def subscribe_jointstates_callback(self,data):

        self.jointstates_pos = data.position
        self.jointstates_vel = data.velocity
        self.jointstates_effort = data.effort

        self.linear_js_pos = data.position[0:1]
        self.linear_js_vel = data.velocity[0:1]

        self.head_js_pos = data.position[1:3] #
        self.head_js_vel = data.velocity[1:3] #

        self.right_arm_js_pos = data.position[4:11]
        self.right_arm_js_vel = data.velocity[4:11]

        self.left_arm_js_pos = data.position[11:18]
        self.left_arm_js_vel = data.velocity[11:18]


    def subscribe_jointstates(self):
        rospy.Subscriber("/joint_states", JointState,
                         self.subscribe_jointstates_callback)

    def get_lpose(self):
        return self.lpose
    def get_rpose(self):
        return self.rpose
    def subscribe_lpose_callback(self,data):
        self.lpose=[
            data.pose.position.x,#xyz xyzw
            data.pose.position.y,#xyz xyzw
            data.pose.position.z,#xyz xyzw
            data.pose.orientation.x,#xyz xyzw
            data.pose.orientation.y,#xyz xyzw
            data.pose.orientation.z,#xyz xyzw
            data.pose.orientation.w]#xyz xyzw
    def subscribe_rpose_callback(self,data):
        self.rpose=[
            data.pose.position.x,#xyz xyzw
            data.pose.position.y,#xyz xyzw
            data.pose.position.z,#xyz xyzw
            data.pose.orientation.x,#xyz xyzw
            data.pose.orientation.y,#xyz xyzw
            data.pose.orientation.z,#xyz xyzw
            data.pose.orientation.w]#xyz xyzw
    def _subscribe_eepose(self):
        rospy.Subscriber("/movo/left_arm/pose", PoseStamped,
                         self.subscribe_lpose_callback)
        rospy.Subscriber("/movo/right_arm/pose", PoseStamped,
                         self.subscribe_rpose_callback)

    def tool_dist(self,l1,l2):
        dist = sum([(v - l1[i]) ** 2 for i, v in enumerate(l2)]) ** 0.5
        return dist

    class Decorators(object):
        @classmethod
        def L_Action(self,fun):
            def do_action(self,*args,**kwargs):
                print("Call->",str(fun.__name__)," : ",args,kwargs)
                fun(self,*args,**kwargs)
            return do_action

        @classmethod
        def Rename(self,fun):
            def do_action(self,*args,**kwargs):
                print("Renamed Call->",str(fun.__name__)," : ",args,kwargs)
                fun(self,*args,**kwargs)
            return fun

    @Decorators.Rename
    def set_grippers(self,*args,**kwargs):
        return self._L0_dual_set_gripper(*args,**kwargs)
    @Decorators.Rename
    def grip(self,rl,v):
        return self._L0_gripper(rl,v)
    @Decorators.Rename
    def arm_cart_move(self,arm,pos,orn,maxforce,wait=True,hard=False):
        return self._L0_single_task_move_safe(arm,pos,orn,maxforce,wait,hard)
    @Decorators.Rename
    def arms_cart_move(self,*args,**kwargs):
        return self._L0_single_task_move_safe(*args,**kwargs)
    @Decorators.Rename
    def single_move_relate(self,arm,move,maxforce,time,wait=True,hard=False):
        return self._L1_single_task_move_safe_relate(arm,move,maxforce,time,wait,hard)
    @Decorators.Rename
    def dual_move_relate(self,rmove,lmove,time,rmaxforce,lmaxforce,wait=True,hard=False):
        self._L0_dual_task_move_safe_relate(rmove,lmove,time,rmaxforce,lmaxforce,wait,hard)
    @Decorators.Rename
    def go_upper_default_jp(self,posdict,hard,duration=30,f=15):
        pos=posdict
        if hard:
            f=1000
        self._L0_upper_jp_move_safe(jpl=pos["l"], jpr=pos["r"], jplinear=pos["lin"],
                                   jph=pos["h"], wait=True, hard=False,
                                   lforce=[f for i in range(6)],
                                   rforce=[f for i in range(6)],
                                   duration=duration
                                   )

    def get_jp_dict(self):
        dic = {
            'h': self.head_js_pos,
            'lin': self.linear_js_pos[0],
            "l": self.left_arm_js_pos,
            "r": self.right_arm_js_pos
        }
        print(dic)
        return dic

    @Decorators.L_Action
    def _L1_single_task_move_safe_relate(self,arm,move,maxforce,time,wait=True,hard=False):
        assert arm in ["left","right"]
        if hard:
            maxforce=[1000 for i in range(6)]


        rmove=[0,0,0]
        lmove=[0,0,0]
        time=time
        rmaxforce=lmaxforce=[1000 for i in range(6)]

        if arm =="right":
            rmove=move
            rmaxforce=maxforce
        elif arm =="left":
            lmove=move
            lmaxforce=maxforce


        # def _L0_dual_task_move_safe_relate(self, rmove, lmove, time, rmaxforce, lmaxforce, wait=True, h

        self._L0_dual_task_move_safe_relate(rmove,lmove,time,rmaxforce,lmaxforce,wait,hard)

    @Decorators.L_Action
    def _L0_dual_set_gripper(self,value,wait=True):
        goal=arcmsg.dual_set_gripperGoal
        goal.value=value
        self.done_thr_set_grippers=False
        if wait:
            self.client_L0_dual_set_gripper.send_goal_and_wait(goal)
        else:
            self.client_L0_dual_set_gripper.send_goal(goal)
        # if wait:
        #     self.client_L0_dual_set_gripper.wait_for_result()
        self.done_thr_set_grippers=True
    @Decorators.L_Action
    def _L0_dual_jp_move_safe_relate(self, jp_r, jp_l, lmaxforce, rmaxforce, duration, wait=True,hard=False):
        if hard:
            f=1000
            lmaxforce=rmaxforce=[f for i in range(6)]
        self.done_thr_jprot_re = False
        mygoal = arcmsg.dual_jp_movo_safe_relateGoal()
        mygoal.jp_left_relate = jp_l
        mygoal.jp_right_relate = jp_r
        mygoal.l_max_force = lmaxforce
        mygoal.r_max_force = rmaxforce
        mygoal.duration = duration
        # send a goal
        if (wait):
            self.client_L0_dual_jp_move_safe_relate.send_goal_and_wait(mygoal)
        else:

            self.client_L0_dual_jp_move_safe_relate.send_goal(mygoal)
        pass
        self.done_thr_jprot_re = True
    @Decorators.L_Action
    def _L0_single_task_move_safe(self,arm,pos,orn,maxforce,wait=True,hard=False):
        if hard:
            f=1000
            maxforce=[f for i in range(6)]
        assert arm in ["left","right"]
        self.done_thr_single_task=False
        mygoal = arcmsg.single_task_move_safeGoal()
        arm = 0 if arm=="left" else 1
        mygoal.pos = pos
        mygoal.orn = orn
        mygoal.arm = arm
        mygoal.max_force = maxforce
        # mygoal.duration = duration
        # send a goal
        if(wait):
            self.client_L0_single_task_move_safe.send_goal_and_wait(mygoal)
        else:
            self.client_L0_single_task_move_safe.send_goal(mygoal)

        self.done_thr_single_task=True
    @Decorators.L_Action
    def _L0_dual_task_move_safe_relate(self,rmove,lmove,time,rmaxforce,lmaxforce,wait=True,hard=False):

        if hard:
            f=1000
            lmaxforce=rmaxforce=[f for i in range(6)]

        self.done_thr_taskmov_re=False
        mygoal = arcmsg.dual_task_move_safe_relateGoal()
        mygoal.pos_r=rmove
        mygoal.pos_l=lmove
        mygoal.time=time
        mygoal.r_max_force=rmaxforce
        mygoal.l_max_force=lmaxforce

        # send a goal
        if wait:
            self.client_L0_dual_task_move_safe_relate.send_goal_and_wait(mygoal)
        else:
            self.client_L0_dual_task_move_safe_relate.send_goal(mygoal)
            # self.client_L0_dual_task_move_safe_relate.wait_for_result()

        self.done_thr_taskmov_re=True
    @Decorators.L_Action
    def _L0_upper_jp_move_safe(self,jpl,jpr,jph,jplinear,duration,lforce,rforce,wait=True,hard=False):

        if hard:
            f=1000
            lforce=rforce=[f for i in range(6)]

        mygoal = arcmsg.upper_jp_movo_safeGoal()
        mygoal.jp_left = jpl
        mygoal.jp_right = jpr
        mygoal.jp_head = jph
        mygoal.jp_linear = jplinear
        mygoal.duration = duration
        mygoal.l_max_force = lforce
        mygoal.r_max_force = rforce
        self.client_L0_upper_jp_move_safe.send_goal(mygoal)
        if wait:
            self.client_L0_upper_jp_move_safe.send_goal_and_wait(mygoal)
        else:
            self.client_L0_upper_jp_move_safe.send_goal(mygoal)
    @Decorators.L_Action
    def _L0_gripper(self,rl,value): #0 close # 1 open
        # print "do close gripper"
        assert rl in ["right","left","r","l"]

        arm=0 if rl in ['l',"left"] else 1
        # arm=1 if rl in ['r',"right"]

        mygoal=arcmsg.single_set_gripperGoal
        mygoal.arm=arm
        mygoal.value=value
        self.client_L0_single_set_gripper.send_goal_and_wait(mygoal)
        # v=float(value)*(self.g._gripper_open-self.g._gripper_closed)+self.g._gripper_closed
        # if(rl in ["left","l"]):
        #     self._lgripper.command(v, block=True)
        # elif(rl in ["right","r"]):
        #     self._rgripper.command(v, block=True)

    # deprecated
    def eval_L0_upper_jp_move_safe(self):
        return self.E0_getjp()
    def _L0_dual_jp_move_safe_relate_decodeargs(self,args):
        self._L0_dual_jp_move_safe_relate(jp_r=args[:7],
                                          jp_l=args[7:14],
                                          rmaxforce=args[14:20],
                                          lmaxforce=args[20:26],
                                          duration=args[26])
    def L0_upper_jp_move_safe(self,args):

        #filter_inputs
        # TODO
            # same input or not
        lastargs=self.client_L0_upper_jp_move_safe_last_args
        newargs=args
        goal_state=self.client_L0_upper_jp_move_safe.get_state()
        if(goal_state in [GoalStatus.PREEMPTED,GoalStatus.SUCCEEDED]):
            #force reached maximum
            pass
            # TODO
            # currently, go on sending goal
        else:
            if(not type(lastargs)==type(None)):
                dist=self.tool_dist(newargs,lastargs)
                if(dist<0.01):
                    return
        self.client_L0_upper_jp_move_safe_last_args=newargs[:]
        #decode args
        mygoal = arcmsg.upper_jp_movo_safeGoal()
        mygoal.jp_left = args[7:14]
        mygoal.jp_right = args[0:7]
        mygoal.jp_head = args[15:17]
        mygoal.jp_linear = args[14:15][0]
        mygoal.duration = 300
        mygoal.l_max_force = args[17:23]
        mygoal.r_max_force = args[23:29]

        # send a goal
        self.client_L0_upper_jp_move_safe.send_goal(mygoal)

# Stable Tests
def routine_test_all():
    # A script to test L0_dual_task_move_safe_relate with ICARUS
    force_left = [10 for i in range(6)]
    force_right = [10 for i in range(6)]
    args_input_1 = arc.gval.default_pose_pickready[:] + force_left + force_right
    args_input_2 = arc.gval.default_pose_horizon_rot_before_insert_ready[:] + force_left + force_right

    # gripper test
    arc._L0_dual_set_gripper(1)
    arc._L0_dual_set_gripper(0)
    arc._L0_dual_set_gripper(1)
    arc._L0_dual_set_gripper(0)

    # jo relate move
    arc._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, 0, 0, 0, 0], rmaxforce=[10, 10, 10, 5, 5, 5],
        jp_l=[0, 0, 0, 0, 0, 0, 1.8], lmaxforce=[10, 10, 10, 5, 5, 5],
        duration=3)
    arc._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, 0, 0, 0, -1.8], rmaxforce=[10, 10, 10, 5, 5, 5],
        jp_l=[0, 0, 0, 0, 0, 0, 0], lmaxforce=[10, 10, 10, 5, 5, 5],
        duration=3)
    arc._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, 0, 0, 0, 1.8], rmaxforce=[10, 10, 10, 5, 5, 5],
        jp_l=[0, 0, 0, 0, 0, 0, -1.8], lmaxforce=[10, 10, 10, 5, 5, 5],
        duration=3)

    # task move test
    arc._L0_dual_task_move_safe_relate(
        rmove=[0, 0.1, 0], rmaxforce=[10 for i in range(6)],
        lmove=[0, -0.1, 0], lmaxforce=[10 for i in range(6)],
        time=3)
    arc._L0_dual_task_move_safe_relate(
        rmove=[0, -0.1, 0], rmaxforce=[10 for i in range(6)],
        lmove=[0, +0.1, 0], lmaxforce=[10 for i in range(6)],
        time=3)

    print("motion finished", "_L0_dual_jp_move_safe_relate")
    rospy.spin()

def rountine_tune_pose():
    # arc.client_L0_single_task_move_safe()

    force=50
    # for i in range(3):
    #     print(i)

    arc._L0_single_task_move_safe("right",[0.74,-0.25,1.17],
                                  [0, math.pi / 2, -math.pi / 2],
                                  [force for i in range(6)])
    arc._L0_single_task_move_safe("right",[0.74,-0.25,1.17],
                                  [-0.3, math.pi / 2, -math.pi / 2],
                                  [force for i in range(6)])
    arc._L0_single_task_move_safe("right",[0.74,-0.25,1.17],
                                  [-0.8, math.pi / 2, -math.pi / 2],
                                  [force for i in range(6)])
    arc._L0_single_task_move_safe("right",[0.74,-0.25,1.17],
                                  [-1.5, math.pi / 2, -math.pi / 2],
                                  [force for i in range(6)])
    arc._L0_single_task_move_safe("right",[0.74,-0.25,1.17],
                                  [-2*math.pi+math.pi/2+0.2, math.pi / 2, -math.pi / 2],
                                  [force for i in range(6)])

    # arc.head_js_pos
    # (-0.001517469179816544, -0.5982711315155029)
    # arc.linear_js_pos
    # (0.46091189980506897,)
    # arc.left_arm_js_pos
    # (0.0148455251805375, 1.0166356471590665, 0.6636496236356031, 1.304654122206155, -0.35251198084942814,
    #  -1.3975923189794133, -0.7291213085063291)
    # arc.right_arm_js_pos
    # (1.016942243579562, -1.142176661471238, -0.013962420963067323, -1.5405055325563353, 0.5841524008839754,
    #  1.2464280997132076, 2.7099240682921577)

    # arc._L0_single_task_move_safe("right",[0.74,-0.25,1.17],
    #                               [0.2, math.pi / 2, -math.pi / 2],
    #                               [force for i in range(6)])
    # arc._L0_single_task_move_safe("right",[0.74,-0.25,1.17],
    #                               [0.3, math.pi / 2, -math.pi / 2],
    #                               [force for i in range(6)])
    arc._L0_single_task_move_safe("left",[0.74,0.17,1.1],
                                  [-0.2+math.pi, 0, -math.pi / 2],
                                  [force for i in range(6)])
    arc._L0_single_task_move_safe("left",[0.74,0.17,1.4],
                                  [-0.2+math.pi, 0, -math.pi / 2],
                                  [force for i in range(6)])
    #jp=(0.4597393572330475, -0.0015124091878533363, -0.598263680934906, 0.22986967861652374,
    # 1.723732347134332, -1.003162315730192, 0.5341834760613051,
    #    -1.551505852596962, -0.44545550394486666, 1.1927813827146514, -0.9088847486657059, -0.4251632484133854, 1.1932325222033107, 0.11834501988872326,
    #    1.2943764509273212, 0.39443652799389284, -1.4401347194967749,
    #    -1.310207079395079, 0.8907163779515354, 0.8907163779515354, 0.8907163779515354, 0.8907163779515354, 0.8907163779515354, 0.8907163779515354)


    # arc._L0_dual_jp_move_safe_relate(jp_r=[0,0,0,0,0,0,0],jp_l=[0,0,0,0,0,0,0],
    #                                  lmaxforce=[10,10,10,10,10,10,10])


    # arc._L0_upper_jp_move_safe(
    #     jp_left, jp_right, jp_head, jp_linear, duration, l_max_force, r_max_force, wait=True, hard=True
    # )
def rountine_tune_pose_screw_and_rarm_observe():

    force=25
    arc._L0_single_task_move_safe("right",[0.739, 0.056-0.03, 1.253-0.06],
                                  [-2*math.pi+0.2,
                                   math.pi / 2+math.pi/4+0.1,
                                   -math.pi / 2],
                                  [force for i in range(6)])

    arc._L0_single_task_move_safe("left",[0.799-0.05, 0.178-0.04, 1.5],
                                  [+math.pi, 0-0.1, -math.pi / 2],
                                  [force for i in range(6)])

    # pos=arc.gval.default_pose_screw_rightarm_observe
    # arc.go_upper_default_jp(arc.gval.default_pose_screw_rightarm_observe)
    # arc._L0_upper_jp_move_safe(jpl=pos["l"],jpr=pos["r"],jplinear=pos["lin"],
    #                            jph=pos["h"],wait=True,hard=False,
    #                            lforce=[15 for i in range(6)],
    #                            rforce=[15 for i in range(6)],
    #                            duration=30
    #                            )

def rountine_tune_pose_pickbolt():

    force=15
    arc._L0_single_task_move_safe("right", [0.74, -0.35, 1.17],
                                [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
                                [force for i in range(6)])
    # arc._L0_single_task_move_safe("left",[0.799-0.05, 0.178-0.04, 1.5],
    #                               [+math.pi, 0-0.1, -math.pi / 2],
    #                               [force for i in range(6)])

    # pos=arc.gval.default_pose_screw_rightarm_observe
    # arc.go_upper_default_jp(arc.gval.default_pose_screw_rightarm_observe)
    # arc._L0_upper_jp_move_safe(jpl=pos["l"],jpr=pos["r"],jplinear=pos["lin"],
    #                            jph=pos["h"],wait=True,hard=False,
    #                            lforce=[15 for i in range(6)],
    #                            rforce=[15 for i in range(6)],
    #                            duration=30
    #                            )

def pickbolt():
    # action sequence
    pass
if __name__=="__main__":

    print("Start")

    rospy.init_node("test")
    arc=odyssey_Interface()
    # rountine_tune_pose_screw_and_rarm_observe()
    # rountine_tune_pose_pickbolt()
    # routine_test_all()
    rountine_tune_pose_screw_and_rarm_observe()

    # print(arc.get_rpose())
    print(arc.get_lpose())
    print(arc.get_jp_dict())

    # {'h': (0.001755023142322898, -0.6001875400543213), 'lin': 0.4609679579734802, 'l': (
    # 0.0038434740852024696, 1.0431375621147718, 0.6896942738321421, 1.1683924197678146, -0.32924767083093087,
    # -1.5484046015075648, -0.7592577059806938), 'r': (
    # 0.6128764574532433, -0.8213907855298692, -0.6226357444258932, -1.5857084310026863, 0.6403175361673652,
    # 1.107163416829694, -2.619071925600757)}

    # print(arc.right_arm_js_pos)
    # print(arc.get_jp_dict())
    # arc.grip("l",0)
    # arc.grip("l",1)
    # arc.grip("l",0)
    # arc.grip("l",0)
    # arc.single_move_relate("right",[0,-0.3,0],[20 for i in range(6)],2)
    # rountine_tune_pose_screw_and_rarm_observe()
    # arc.set_grippers(0)
    # arc.set_grippers(0)
    # rountine_tune_pose()
