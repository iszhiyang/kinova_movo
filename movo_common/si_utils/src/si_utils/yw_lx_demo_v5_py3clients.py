# Python3 Interfaces for ARC actions libraries
# It connects to py2 servers, with py3 clients
# Email: yiwen.chen@u.nus.edu
import rospy
import actionlib
import movo_arc_lib.msg as arcmsg
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatus
import threading
import numpy as np




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

class ARC_ACTION_LIB_Interface():
    # simple interface name with L0...
    # ICURAS interface name with i_L0...
    def __init__(self):
        self.gval=GlobalVariables()
        rospy.init_node("ARC_ACTION_LIB_Interface_py3_node")

        self.client_L0_upper_jp_move_safe=myclient(name="L0_upper_jp_move_safe",
                                                   action=arcmsg.upper_jp_movo_safeAction,
                                                   fbmsg=arcmsg.upper_jp_movo_safeFeedback)

        self.client_L0_dual_jp_move_safe_relate=myclient(name="L0_dual_jp_move_safe_relate",
                                                   action=arcmsg.dual_jp_movo_safe_relateAction,
                                                   fbmsg=arcmsg.dual_jp_movo_safe_relateFeedback)

        # self.client_L0_dual_task_move_safe_relate=myclient(name="L0_dual_task_move_safe_relate",
        #                                                    action=arcmsg.dual_task_move_safe_relateAction,
        #                                                    fbmsg=arcmsg.dual_task_move_safe_relateFeedback)

        self.client_L0_dual_set_gripper=myclient(name="L0_dual_set_gripper",
                                                 action=arcmsg.dual_set_gripperAction,
                                                 fbmsg=arcmsg.dual_set_gripperFeedback)

        self.thread_L0_dual_jp_move_safe_relate=threading.Thread()
        self.thread_L0_dual_jp_move_safe_relate_finished=None
        self.client_L0_upper_jp_move_safe_last_args=None
        self.client_L0_dual_jp_move_safe_relate_last_args=None

        rospy.Subscriber("/joint_states", JointState,self._subscribe_jointstates_callback)

        self.thr_jprot_re=threading.Thread()
        self.thr_taskmov_re=threading.Thread()
        self.thr_set_grippers=threading.Thread()
        self.done_thr_jprot_re=None
        self.done_thr_taskmov_re=None
        self.done_thr_set_grippers=None



        self.jp_updated=False
        r=rospy.Rate(10)
        while(not (self.jp_updated and True)):
            r.sleep()


    def start_func_thr(self,funcname,thr,args,doneflag):
        thr=threading.Thread(target=funcname,args=(args,))
        thr.start()
        exec(str(doneflag)+"=False")

    def check_thr_finished(self,doneflag,thr):
        # alive=thr.isAlive()

        doneflag_bool=eval("self."+doneflag)
        # print(doneflag_bool)
        if(type(doneflag_bool) == type(None)):
            print("check_thr_finished: doneflag == none")
            return False
        if(doneflag_bool):
            print("check_thr_finished: doneflag == True")
            # doneflag_pointer=None
            # doneflag_pointer = exec("self." + str(doneflag) + "=None")
            exec("self." + str(doneflag) + "=None")
            return True
        # return None


    def _subscribe_jointstates_callback(self,data):
        self.linear_js_pos = data.position[0:1]
        self.linear_js_vel = data.velocity[0:1]

        self.head_js_pos = data.position[1:3] #
        self.head_js_vel = data.velocity[1:3] #

        self.right_arm_js_pos = data.position[4:11]
        self.right_arm_js_vel = data.velocity[4:11]

        self.left_arm_js_pos = data.position[11:18]
        self.left_arm_js_vel = data.velocity[11:18]

        self.jp=self.right_arm_js_pos+self.left_arm_js_pos+self.linear_js_pos+self.head_js_pos

        self.jp_updated = True

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

    def tool_dist(self,l1,l2):
        dist = sum([(v - l1[i]) ** 2 for i, v in enumerate(l2)]) ** 0.5
        return dist

    def _L0_dual_set_gripper(self,value):
        goal=arcmsg.dual_set_gripperGoal
        goal.value=value
        self.done_thr_set_grippers=False
        self.client_L0_dual_set_gripper.send_goal(goal)
        self.client_L0_dual_set_gripper.wait_for_result()

        self.done_thr_set_grippers=True
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
    def eval_L0_upper_jp_move_safe(self):
        return self.E0_getjp()

    def _L0_dual_jp_move_safe_relate_decodeargs(self,args):
        self._L0_dual_jp_move_safe_relate(jp_r=args[:7],
                                          jp_l=args[7:14],
                                          rmaxforce=args[14:20],
                                          lmaxforce=args[20:26],
                                          duration=args[26])
    def _L0_dual_jp_move_safe_relate(self,jp_r,jp_l,lmaxforce,rmaxforce,duration,wait=True):
        self.done_thr_jprot_re=False
        mygoal = arcmsg.dual_jp_movo_safe_relateGoal()
        mygoal.jp_left_relate = jp_l
        mygoal.jp_right_relate = jp_r
        mygoal.l_max_force = lmaxforce
        mygoal.r_max_force = rmaxforce
        mygoal.duration = duration


        # send a goal
        self.client_L0_dual_jp_move_safe_relate.send_goal(mygoal)
        if(wait):
            self.client_L0_dual_jp_move_safe_relate.wait_for_result()
        pass
        self.done_thr_jprot_re=True
    def _L0_dual_task_move_safe_relate_decodeargs(self,args):

        # rmove = [0, 0.1, 0]
        # lmove = [0, -0.1, 0]
        # rmaxforce = [10 for i in range(6)]
        # lmaxforce = [10 for i in range(6)]
        # time = 3
        # args = rmove + lmove + rmaxforce + lmaxforce + [time]

        self._L0_dual_task_move_safe_relate(
            rmove=args[0:3], rmaxforce=args[6:12],
            lmove=args[3:6], lmaxforce=args[12:18],
            time=args[18]
        )
    def _L0_dual_task_move_safe_relate(self,rmove,lmove,time,rmaxforce,lmaxforce):

        self.done_thr_taskmov_re=False

        mygoal = arcmsg.dual_task_move_safe_relateGoal()
        mygoal.pos_r=rmove
        mygoal.pos_l=lmove
        mygoal.time=time
        mygoal.r_max_force=rmaxforce
        mygoal.l_max_force=lmaxforce

        # send a goal
        self.client_L0_dual_task_move_safe_relate.send_goal(mygoal)
        self.client_L0_dual_task_move_safe_relate.wait_for_result()

        self.done_thr_taskmov_re=True

    def deprecated_L0_dual_jp_move_safe_relate(self,args):

        def get_round_list(target_list):
            # target jp beyond one round
            l=[0 for i in range(len(target_list))]
            for i in range(len(target_list)):
                if(target_list[i]>3.1415):
                    l[i]=1
                if(target_list[i]<3.1415):
                    l[i]=-1
            return l
        def satisfied(self, final_jp_r, final_jp_l, current_jp_r, current_jp_l):
            final = final_jp_r + final_jp_l
            now = current_jp_r + current_jp_l
            d = self.tool_dist(final, now) / 2
            # print("Final,",final,"\nnow",now)
            if (d <= 0.05):
                print(__name__, "d={}".format(d), "jp move relate ->  safisfied target goal, stop rot")
                return True
            else:
                print(__name__, "d={}".format(d), "jp move relate -> not safisfied target goal, keep rot")
                return False
        def clip(l):
            for i in range(len(l)):
                r = l[i]
                while (r > 3.1415):
                    if (r > 3.1415):
                        r -= 3.1415 * 2
                    l[i] = r
                while (r < -3.1415):
                    if (r < -3.1415):
                        r += 3.1415 * 2
                    l[i] = r
            return l
        def calculate_remain(self, final_jp_r, final_jp_l, current_jp_r, current_jp_l,
                             round_r,round_l):


            # final_jp_r=clip(final_jp_r)
            # final_jp_l=clip(final_jp_l)

            remain_l = [-current_jp_l[i] + final_jp_l[i] for i in range(len(final_jp_l))]
            remain_r = [-current_jp_r[i] + final_jp_r[i] for i in range(len(final_jp_r))]

            for i in range(len(round_r)):
                remain_r[i]=remain_r[i]+3.1415*2*remain_r[i]
                remain_l[i]=remain_l[i]+3.1415*2*remain_l[i]

            print("c l", current_jp_l)
            print("c r", current_jp_r)
            print("remain l", remain_l)
            print("remain r", remain_r)
            print("tar l", final_jp_l)
            print("tar r", final_jp_r)
            return remain_l, remain_r

        newargs = args
        ## same input or not
        # lastargs=self.client_L0_dual_jp_move_safe_relate_last_args
        jp_left_relate = args[7:14]
        jp_right_relate = args[0:7]
        current_jp_l=self.E0_getjp_arm("l")
        current_jp_r=self.E0_getjp_arm("r")
        final_jp_l=[current_jp_l[i]+jp_left_relate[i] for i in range(len(jp_left_relate))]
        final_jp_r=[current_jp_r[i]+jp_right_relate[i] for i in range(len(jp_right_relate))]

        print(final_jp_l)
        print(final_jp_r)
        round_list_l=get_round_list(final_jp_l)
        round_list_r=get_round_list(final_jp_r)
        final_jp_l= clip(final_jp_l)
        final_jp_r= clip(final_jp_r)

        while(not satisfied(self,final_jp_r,final_jp_l,current_jp_r,current_jp_l)):
            # GOTO Target Joint
            current_jp_l=self.E0_getjp_arm('l')
            current_jp_r=self.E0_getjp_arm('r')
            remain_l,remain_r=calculate_remain(self,
                final_jp_r,final_jp_l,
                current_jp_r,current_jp_l,round_list_r,round_list_l)
            # print("fl", final_jp_l)
            # print("fr", final_jp_r)
            # print("cl", current_jp_l)
            # print('cr', current_jp_r)
            # self.client_L0_dual_jp_move_safe_relate_last_args=newargs[:]
            #decode args
            mygoal = arcmsg.dual_jp_movo_safe_relateGoal()
            mygoal.jp_left_relate = remain_l
            mygoal.jp_right_relate = remain_r
            mygoal.l_max_force = args[14:20]
            mygoal.r_max_force = args[20:26]
            mygoal.duration = args[26]

            #update relate move start

            # send a goal
            self.client_L0_upper_jp_move_safe_relate.send_goal(mygoal)
            self.client_L0_upper_jp_move_safe_relate.wait_for_result()
            print("goal finished")
        self.thread_L0_dual_jp_move_safe_relate_finished=True
    def deprecated_eval_L0_dual_jp_move_safe_relate(self):

        alive = self.thread_L0_dual_jp_move_safe_relate.isAlive()
        if (type(self.thread_L0_dual_jp_move_safe_relate_finished) == type(None)):
            return False
        if (not alive):
            pass
            # print("ie_status_dual_jp_rot not alive")
        if (self.thread_L0_dual_jp_move_safe_relate_finished):
            print("dual joints rot finished")
            self.thread_L0_dual_jp_move_safe_relate_finished = None
            return True
        return None
    def deprecated_L0_dual_jp_move_safe_relate(self,args):

        done = self.thread_L0_dual_jp_move_safe_relate_finished
        allow= (type(done)==type(None))

        if (allow):
            self.thread_L0_dual_jp_move_safe_relate_finished = False
            self.thread_L0_dual_jp_move_safe_relate=threading.Thread(target=self._L0_dual_jp_move_safe_relate,args=(args,))
            self.thread_L0_dual_jp_move_safe_relate.start()
        else:
            pass




class ICARUS_interface(object):
    """
    ICARUS_interface Introduction:

    Architecture Flow:
        server_py2 -> ros -> client_py3 -> ICARUS

    4 motion interface we have:
        move_relate_jp
        move_abs_jp
        move_relate_task
        set_grippers

    Notes:
    1. Safety consideration. Force Detected cancellable motions.
            For protecting robot and user, if the external force is detected over the thresh,
            the motion will be canceled. Thresh can be given by user.
        move_relate_jp
        move_abs_jp
        move_relate_task

    2. move_abs_task not implemented. Due to that it is dangerous and collides easily,
    and can not do dual arm synchronized moving. A safer version is under developing.

    3. Dual Arms Synchronized Move supported: (Dual Move simultaneously)
        move_relate_jp
        move_abs_jp
        move_relate_task
        set_grippers

    4. True/False Motion:
            These motions Do Not support using goal jp or pos values to eval its status.
            It returns True or False (with eval function), to tell this motion finish or not.
        move_relate_jp
        move_relate_task
        set_grippers

    5. All motions have solved with py2/py3 conflicts.

    """
    def __init__(self):
        self.arc=ARC_ACTION_LIB_Interface()
    def move_relate_jp(self,args):
        """

        :param args: list[float x (7+7+6+6+1)]

                    Example:
                        jp_r = [0, 0, 0, 0, 0, 0, 3.1415 / 2]
                        jp_l = [0, 0, 0, 0, 0, 0, -3.1415 / 2]
                        rmaxforce = [10, 10, 10, 10, 10, 10]
                        lmaxforce = [10, 10, 10, 10, 10, 10]
                        duration = 3
                        args = jp_r + jp_l + rmaxforce + lmaxforce + [duration]

        :return:
        """

        done = self.arc.done_thr_jprot_re
        allow= (type(done)==type(None))
        if(allow):
            arc.start_func_thr(funcname=arc._L0_dual_jp_move_safe_relate_decodeargs,
                               thr=arc.thr_jprot_re,
                               args=(args),
                               doneflag="done_thr_jprot_re")
        pass
    def eval_move_relate_jp(self):
        return self.arc.check_thr_finished(
            doneflag="done_thr_jprot_re",
            thr=self.arc.thr_jprot_re)

    def move_abs_jp(self,args):
        """
        it controls 17 joints in the upper body.
        Including
        7 in the right_arm ,
        7 in the left_arm,
        2 in the head,
        1 in the body.
        :param args: list[float x (29)]
                    Example:
                        mygoal.jp_left = args[7:14]
                        mygoal.jp_right = args[0:7]
                        mygoal.jp_head = args[15:17]
                        mygoal.jp_linear = args[14:15][0]
                        mygoal.duration = 300
                        mygoal.l_max_force = args[17:23]
                        mygoal.r_max_force = args[23:29]
        :return:
        """
        self.arc.L0_upper_jp_move_safe(args)
        pass
    def eval_move_abs_jp(self):
        return self.arc.eval_L0_upper_jp_move_safe()
        pass

    def move_relate_task(self,args):
        """
        Dual Arms move simultaneously in the Cartesian space.
        :param args:  args= list[float x (19)]
                        Example:
                        rmove=args[0:3],
                        rmaxforce=args[6:12],
                        lmove=args[3:6],
                        lmaxforce=args[12:18],
                        time=args[18]
        :return:
        """
        done = self.arc.done_thr_taskmov_re
        allow= (type(done)==type(None))
        if(allow):
            arc.start_func_thr(funcname=self.arc._L0_dual_task_move_safe_relate_decodeargs,
                               thr=self.arc.thr_taskmov_re,
                               args=(args),
                               doneflag="done_thr_taskmov_re")
        pass
        pass
    def eval_move_relate_task(self):

        return self.arc.check_thr_finished(
            doneflag="done_thr_taskmov_re",
            thr=self.arc.thr_taskmov_re)


    def set_grippers(self,args):
        '''
        Set Two Grippers at the same time
        :param args: 0<int_value<1,  1 for fully open, 0 for fully close.
        :return:
        '''
        done = self.arc.done_thr_set_grippers
        allow= (type(done)==type(None))
        if(allow):
            self.arc.start_func_thr(funcname=self.arc._L0_dual_set_gripper,
                               thr=self.arc.thr_set_grippers,
                               args=(args),
                               doneflag="done_thr_set_grippers")

    def eval_set_grippers(self):
        return self.arc.check_thr_finished(
            doneflag="done_thr_set_grippers",
            thr=self.arc.thr_set_grippers)
        pass

# Stable Tests
def routine_test_all():
    # A script to test L0_dual_task_move_safe_relate with ICARUS
    force_left = [10 for i in range(6)]
    force_right = [10 for i in range(6)]
    args_input_1 = arc.gval.default_pose_pickready[:] + force_left + force_right
    args_input_2 = arc.gval.default_pose_horizon_rot_before_insert_ready[:] + force_left + force_right

    args_input = args_input_1
    motion(arc.L0_upper_jp_move_safe, args_input, args_input[:17], arc.eval_L0_upper_jp_move_safe, 1)
    args_input = args_input_2
    motion(arc.L0_upper_jp_move_safe, args_input, args_input[:17], arc.eval_L0_upper_jp_move_safe, 1)
    args_input = args_input_1
    motion(arc.L0_upper_jp_move_safe, args_input, args_input[:17], arc.eval_L0_upper_jp_move_safe, 1)
    print("motion finished", "L0_upper_jp_move_safe")

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

def routine_test_icarus():
    # A script to test L0_dual_task_move_safe_relate with ICARUS
    force_left = [10 for i in range(6)]
    force_right = [10 for i in range(6)]
    args_input_1 = arc.gval.default_pose_pickready[:] + force_left + force_right
    args_input_2 = arc.gval.default_pose_horizon_rot_before_insert_ready[:] + force_left + force_right
    args_input = args_input_1
    goal = args_input[:17]
    motion(icarus.move_abs_jp, args_input, goal, icarus.eval_move_abs_jp, 1)
    goal = args_input_2[:17]
    motion(icarus.move_abs_jp, args_input_2, goal, icarus.eval_move_abs_jp, 1)
    goal = args_input[:17]
    motion(icarus.move_abs_jp, args_input, goal, icarus.eval_move_abs_jp, 1)

    print("motion finished", "L0_upper_jp_move_safe")

    ## Test Pass
    # # icarus dual jp move relate
    jp_r = [0, 0, 0, 0, 0, 0, 3.1415 / 2]
    jp_l = [0, 0, 0, 0, 0, 0, -3.1415 / 2]
    rmaxforce = [10, 10, 10, 10, 10, 10]
    lmaxforce = [10, 10, 10, 10, 10, 10]
    duration = 3
    args_move_relate_jp = jp_r + jp_l + rmaxforce + lmaxforce + [duration]
    motion(icarus.move_relate_jp,args_move_relate_jp,True,icarus.eval_move_relate_jp,2)
    print("motion finished")

    ## Test Pass
    # # gripper
    value=1
    motion(icarus.set_grippers,value,True,icarus.eval_set_grippers,2)
    value=0
    motion(icarus.set_grippers,value,True,icarus.eval_set_grippers,2)
    value=1
    motion(icarus.set_grippers,value,True,icarus.eval_set_grippers,2)
    value=0
    motion(icarus.set_grippers,value,True,icarus.eval_set_grippers,2)


    ## task move relate
    rmove=[0, 0.1, 0]
    lmove=[0, -0.1, 0]
    rmaxforce = [10 for i in range(6)]
    lmaxforce = [10 for i in range(6)]
    time=3
    args=rmove+lmove+rmaxforce+lmaxforce+[time]
    # icarus.arc._L0_dual_task_move_safe_relate_decodeargs(args)
    # print("_L0_dual_task_move_safe_relate_decodeargs finished")
    motion(icarus.move_relate_task,args,True,icarus.eval_move_relate_task,2)
    print("finished")
    rospy.spin()

if __name__=="__main__":
    print("Start")

    icarus=ICARUS_interface()
    arc=icarus.arc
    routine_test_icarus()
    # routine_test_all()