#!/usr/bin/env python 
#encoding:utf8 

"""
author 
    
        yxm  created in 2016-08
        
"""

from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import rospy
import tf
import math
import actionlib
from smach import State, StateMachine, Concurrence, Container, UserData
from smach.concurrence import Concurrence
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
import subprocess
            
class Speak(State):
    def __init__(self,string="",mode=0):
        State.__init__(self, outcomes=["succeeded"])
        self.string=string
        self.mode=mode
        self.pub=rospy.Publisher("tts_data", String, queue_size=100)
        
    def execute(self, userdata):
        global NAME,PERSON_ID,OBJECT,targetname
        if self.mode == 1:
            self.name = NAME[PERSON_ID]
            self.object = targetname[OBJECT[PERSON_ID]]
            self.string = 'Your name is' + self.name + ',you need a' + self.object
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
        angle=[0.3,0.5]
        for i in range(5):
            for j in angle :
                goal = Point()
                goal.y = j
                self.angle_srv.call(goal)
                sleep(1)
                for k in range(2) :
                    self.object_info = self.xm_findobject.call()
                    if len(self.object_info.objects) > 0 :
                        for i in self.object_info.objects :
                            if i.name == 'ASaMu':
                                object_pos = i.pos
                                object_name = i.name
                                print object_name
                                print object_pos
                                object_pos.header.stamp = rospy.Time()
                                object_pos.header.frame_id = 'camera_link'
                                ps = self.tf_listener.transformPoint('arm_base', object_pos)
                                print ps
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
        return 'aborted'        

class FindAgain(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.xm_findobject = rospy.ServiceProxy('Find_Object', xm_ObjectDetect)
        self.angle_srv= rospy.ServiceProxy('xm_robot/simple_head_controller/look',xm_Look)
        self.base_service_client = rospy.ServiceProxy('move', xm_Move)
        self.tf_listener = tf.TransformListener()

    def execute(self, ud):
        global object_name,object_pos,object_try
        goal = Point()
        goal.y = 0.5
        self.angle_srv.call(goal)
        sleep(1)
        y = 0.00
        for i in range(5) :
            for j in range(2) :
                self.object_info = self.xm_findobject.call()
                if len(self.object_info.objects) > 0 :
                    object_pos = self.object_info.objects[0].pos
                    object_name = self.object_info.objects[0].name
                    print object_name
                    print object_pos
                    object_pos.header.frame_id = 'camera_link'
                    object_pos.header.stamp = rospy.Time()
                    ps = self.tf_listener.transformPoint('arm_base', object_pos)
                    rospy.sleep(1.0)
                    return 'succeeded'
            if y >= 0.3 :
                y = - 2*y
            else :
                y += 0.03
                self.base_service_client.call(0,y,0)
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
            ps.point.x += 0.07
            ps.point.z += 0.03
            return self.pick(ps)
        elif self.mode == 1 :
            ps.point.x += 0.05
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
