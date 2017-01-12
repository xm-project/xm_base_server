#!/usr/bin/env python 
#encoding:utf8 

"""
author

            yxm
            created in 2016-08
            
"""

import rospy
import tf
import actionlib
import math
from target import target
from smach import State, StateMachine, Concurrence, Container, UserData,Iterator
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from rospy.exceptions import ROSInterruptException
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from people_msgs.msg import PositionMeasurementArray,PositionMeasurement
from actionlib_msgs.msg import GoalStatus
from time import sleep
from std_msgs.msg import String,Int32
import threading
from xm_msgs.srv import *
from xm_msgs.msg import *
from collections import OrderedDict
import subprocess
import PyKDL
from math import *
from std_srvs.srv._Trigger import *
from copy import deepcopy
from smach.user_data import UserData
from geometry_msgs.msg._PointStamped import PointStamped

#shopping 
# status=0x07
object_dic = []
ShelfPoint = []
object_name =''
object_pos = PointStamped()
slf = 0

            
class Speak(State):
    def __init__(self,string="",mode=0):
        State.__init__(self, outcomes=["succeeded"])
        self.string=string
        self.mode=mode
        self.pub=rospy.Publisher("tts_data", String, queue_size=100)
        
    def execute(self, userdata):
        global slf,object_dic,target
        if self.mode == 2:
            self.string = 'i find a ' + target[object_dic[slf]]['name']
        self.pub.publish(self.string)
        rospy.sleep(2)
        return "succeeded"      
            
class Wait(State):
    def __init__(self,rec=0):
        State.__init__(self, outcomes=["succeeded"])
        self.rec=rec
        pass

    def execute(self, userdata):
        #test
        rospy.sleep(self.rec)
        return "succeeded"

class FindObject(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','again'])
        self.xm_findobject = rospy.ServiceProxy('Find_Object', xm_ObjectDetect)
        self.angle_srv= rospy.ServiceProxy('xm_robot/simple_head_controller/look',xm_Look)
        self.base_service_client = rospy.ServiceProxy('move', xm_Move)
        self.tf_listener = tf.TransformListener()

    def execute(self, ud):
        global object_name,object_pos
        angle=[0,0.2]
        for i in range(5):
            for j in angle :
                goal = Point()
                goal.y = j
                self.angle_srv.call(goal)
                sleep(1)
                for k in range(2) :
                    self.object_info = self.xm_findobject.call()
                    if len(self.object_info.objects) > 0 :
                        object_name = self.object_info.objects[0].name
                        object_pos = self.object_info.objects[0].pos
                        print object_name
                        print object_pos
                        object_pos.header.stamp = rospy.Time()
                        object_pos.header.frame_id = 'camera_link'
                        ps = self.tf_listener.transformPoint('arm_base', object_pos)
                        print ps
                        if ps.point.z >= -0.1 and ps.point.x <= 1.5:
                            dis = sqrt(ps.point.x*ps.point.x + ps.point.y * ps.point.y + ps.point.z * ps.point.z)
                            print dis.real
                            if dis.real<0.7 or 0.95<dis.real :
                                ps.point.x -= 0.8
                                ps.point.y *= 0.9
                                rospy.loginfo(ps.point)
                                rs = self.base_service_client.call(ps.point.x, 0, 0)
                                rospy.sleep(1.0)
                                rs_ = self.base_service_client.call(0, ps.point.y,0)
                                rospy.sleep(1.0)
                                return'again'
                            return 'succeeded'
            self.base_service_client.call(0.05,0,0)
            sleep(2)
        return 'aborted'        

class FindAgain(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.xm_findobject = rospy.ServiceProxy('Find_Object', xm_ObjectDetect)
        self.angle_srv= rospy.ServiceProxy('xm_robot/simple_head_controller/look',xm_Look)
        self.base_service_client = rospy.ServiceProxy('move', xm_Move)
        self.tf_listener = tf.TransformListener()

    def execute(self, ud):
        global object_name,object_pos
        goal = Point()
        goal.y = 0.05
        self.angle_srv.call(goal)
        sleep(1)
        y = 0.00
        for i in range(5) :
            for j in range(2) :
                self.object_info = self.xm_findobject.call()
                if len(self.object_info.objects) > 0 :
                    object_name = self.object_info.objects[0].name
                    object_pos = self.object_info.objects[0].pos
                    print object_name
                    print object_pos
                    object_pos.header.frame_id = 'camera_link'
                    object_pos.header.stamp = rospy.Time()
                    ps = self.tf_listener.transformPoint('arm_base', object_pos)
                    if abs(ps.point.y.real) >=0.15 :
                        self.base_service_client.call(0,ps.point.y,0)
                        ps.point.y = 0
                        object_pos = self.tf_listener.transformPoint('camera_link', ps)
                        for i in range(5):
                            self.object_again = self.xm_findobject.call()
                            opject_pos = self.object_again.objects[0].pos
                            rospy.sleep(1.0)
                            return 'succeeded'
                    return 'succeeded'
            if y >= 0.03 :
                y = - 2*y
            else :
                y += 0.03
            self.base_service_client.call(0,y,0)
            sleep(2)
        return 'aborted'
                    
class ArmCmd(State):
    def __init__(self,mode=0):
        State.__init__(self, outcomes=[ 'succeeded', 'aborted'])
        self.xm_pick_place_client = rospy.ServiceProxy('xm_pick_or_place', xm_PickOrPlace)
        self.base_service_client = rospy.ServiceProxy('move', xm_Move)
        self.tf_listener = tf.TransformListener()
        self.mode = mode
        
    def execute(self, ud):
        rospy.loginfo('Moving arm')
        global object_pos
        '''
        if ud.mode == 'pick' :
            ud.n_y = -0.15
            return self.pick(ud.arm_goal)
        elif ud.mode == 'place':
            return self.place(ud.arm_goal)
            '''
        ps = self.tf_listener.transformPoint('arm_base', object_pos)
        if self.mode == 0 :
            return self.pick(ps)
        elif self.mode == 1 :
            ps.point.z = 0.2
            return self.place(ps)
            
    def pick(self, goal):
        rospy.loginfo('Pick object: ' + str(goal))
        g = xm_PickOrPlaceRequest()
        g.action = 1
        g.goal_position = goal
        rs = self.xm_pick_place_client.call(g)
        sleep(2)
        if rs.result == True:
            return 'succeeded'
        else:
            return 'aborted'
        
    def place(self, goal):
        rospy.loginfo('Place object: ' + str(goal))
        g = xm_PickOrPlaceRequest()
        g.action = 2
        g.goal_position = goal
        rs = self.xm_pick_place_client.call(g)
        if rs.result == True:
            return 'succeeded'
        else:
            return 'aborted'                

class Get_Odom(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(), rospy.Duration(60))
        
    def execute(self, userdata):
        global ShelfPoint
        (position, orientation) = self.get_odom()
        ShelfPoint.append(Pose(position,orientation))
        return 'succeeded'
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), Quaternion(*rot))
        
class Move(State):
    def __init__(self,shelf = 0):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        self.shelf = shelf
        
    def execute(self, UserData):
        global ShelfPoint,slf
        if self.shelf == 0:
            self.shelf = slf
#             slf += 1
        if self.shelf == 4:
            ShelfPoint[4].position.x += 0.05
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose=ShelfPoint[self.shelf]
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result(rospy.Duration(25))
        if self.move_base.get_goal_status_text() == 'Goal reached.' :
            return 'succeeded'
        return 'aborted'
        
class main():
    def __init__(self):
        rospy.init_node("shopping")
        rospy.on_shutdown(self.shutdown)
        global object_name,slf
        
        self.sm_remember = StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid'])
        with self.sm_remember:
            self.sm_point_iterator = Iterator(outcomes =['succeeded','preempted','aborted'],
                                            input_keys = [],
                                            it = lambda: range(0, 4),
                                            output_keys = [],
                                            it_label = 'index',
                                            exhausted_outcome = 'succeeded')
            with self.sm_point_iterator :
                self.sm_point = StateMachine(outcomes = ['succeeded','aborted','preempted','valid','invalid','continue'])
                with self.sm_point :
                    StateMachine.add('GET_OBJECT',MonitorState('task_coming',xm_Task,self.object_cb),transitions = {'invalid':'REMEMBER_POINT','valid':'GET_OBJECT'})
                    StateMachine.add('REMEMBER_POINT', Get_Odom(),transitions={'succeeded':'SPEAK'})
                    StateMachine.add('SPEAK',Speak(string='i remembered this place'),transitions={'succeeded':'continue'})
                Iterator.set_contained_state('sm_point', self.sm_point, loop_outcomes=['continue'])
                
            StateMachine.add('PUT',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 3),response_cb = self.put_cb),transitions={'succeeded':'START','aborted':'PUT'})
            StateMachine.add('START', MonitorState("people" , xm_People, self.start_cb) , transitions={'invalid':'MOVE','valid':'START'})
            StateMachine.add('MOVE',ServiceState('follow', Trigger, response_cb = self.base_cb, outcomes=['succeeded' , 'failed']) , transitions={'succeeded':'Point_Iterator','failed':'MOVE'})
            StateMachine.add('Point_Iterator', self.sm_point_iterator,transitions={'succeeded':'STOP_OR_NOT','aborted':'','preempted':''})
            StateMachine.add('STOP_OR_NOT',MonitorState('task_coming',xm_Task,self.stop_cb),transitions = {'invalid':'STOP','valid':'STOP_OR_NOT'})
            StateMachine.add('STOP',ServiceState('follow', Trigger, response_cb = self.base_cb, outcomes=['succeeded' , 'failed']) , transitions={'succeeded':'REMEMBER_POINT','failed':'STOP'})            
            StateMachine.add('REMEMBER_POINT', Get_Odom(),transitions={'succeeded':'SPEAK1'})
            StateMachine.add('SPEAK1',Speak(string='i arrived to the cash'),transitions={'succeeded':''})
                        
        self.sm_move_pick = StateMachine(outcomes = ['succeeded', 'aborted', 'preempted', 'valid', 'invalid','next'])
        with self.sm_move_pick :
            self.sm_grasp=StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid'])
            with self.sm_grasp:
                StateMachine.add('READY',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 0),response_cb = self.put_cb),transitions={'succeeded':'FIND_OBJECT','aborted':'READY'})
                StateMachine.add('FIND_OBJECT',FindObject(), transitions={'succeeded':'TALK','again':'FIND_AGAIN','aborted':'PUT'})
                StateMachine.add('FIND_AGAIN',FindAgain(), transitions={'succeeded':'TALK','aborted':'PUT'})
                StateMachine.add('TALK',Speak(mode=2),transitions={'succeeded':'GRASP'})
                StateMachine.add('GRASP',ArmCmd(mode=0),transitions={'succeeded':'PUT','aborted':'PUT'})
                StateMachine.add('PUT',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 3),response_cb = self.put_cb),transitions={'succeeded':'','aborted':'PUT'})
            
            StateMachine.add('ASK',Speak(string='what do you need'),transitions={'succeeded':'OBJECT_NAME'})
            StateMachine.add('OBJECT_NAME',MonitorState('task_coming',xm_Task,self.object_name_cb),transitions = {'invalid':'BACK2POINT','valid':'OBJECT_NAME'})
            StateMachine.add('BACK2POINT',Move(),transitions={'succeeded':'WAIT','aborted':'MOVE2CASH'})
            StateMachine.add('WAIT',Wait(rec=2),transitions={'succeeded':'SPEAK1'})              
            StateMachine.add('SPEAK1',Speak(string='i can not find the object'),transitions={'succeeded':'MOVE2CASH'})              
          #  StateMachine.add('GRASP',self.sm_grasp,transitions={'succeeded':'MOVE2CASH','aborted':'MOVE2CASH'})
            StateMachine.add('MOVE2CASH',Move(shelf=4),transitions={'succeeded':'SPEAK','aborted':'MOVE2CASH'})
            StateMachine.add('SPEAK',Speak(string='i arrived to cash'),transitions={'succeeded':'PUT'})
            StateMachine.add('PUT_DOWN',ArmCmd(mode=1),transitions={'succeeded':'PUT','aborted':'PUT'})
            StateMachine.add('PUT',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 3),response_cb = self.put_cb),transitions={'succeeded':'','aborted':''})

        self.sm_remember.execute()
        
        for i in range(3):
             self.sm_move_pick.execute()


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)        
        pass      

    def start_cb(self,userdata,msg):
        for i in msg.person:
            if i.pose.data == 'raising':
                return False
        return True
    
    def object_cb(self,UserData,msg):
        global object_dic
        if msg.task is not None :
            print (msg.task)
            object_dic.append(msg.task)
            return False
        else:
            return True
        
    def object_name_cb(self,UserData,msg):    
        global slf,object_dic,object_name,target
        if msg.place == 0x08 :
            object_name = deepcopy(target[msg.task]['name'])
            for i in range(len(object_dic)) :
                if msg.task == object_dic[i]:
                    slf = i
                    return False
        return True 
        
    def stop_cb(self,userdata,msg):
        if msg.place == 0x06 :
            return False
        else:
            return True    
        
    def base_cb(self, userdata, responce) :
        if(responce.success==True) :
            return 'succeeded'
        else :
            return 'failed'
    
    def put_cb(self,UserData,response):
        if response.result ==True:
            return 'succeeded'
        else :
            return 'aborted'     
        
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
