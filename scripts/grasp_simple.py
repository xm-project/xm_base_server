#!/usr/bin/env python 
#encoding:utf8 

"""
author 
    
        yxm  created in 2016-07
"""

import rospy
import tf
import actionlib
import math
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from rospy.exceptions import ROSInterruptException
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from people_msgs.msg import PositionMeasurementArray,PositionMeasurement
from actionlib_msgs.msg import GoalStatus 
from time import sleep
from std_msgs.msg import String,Int32,Bool
from std_srvs.srv import *
import threading
from xm_msgs.srv import *
from xm_msgs.msg import *
from collections import OrderedDict
import subprocess
from math import *

object_pos = []
object_name = ''
flag = 0
            
class Wait(State):
    def __init__(self,rec=0):
        State.__init__(self, outcomes=["succeeded"])
        self.rec=rec
        pass

    def execute(self, userdata):
        #test
        rospy.sleep(self.rec)
        return "succeeded"

class Speak(State):
    def __init__(self,string = ''):
        State.__init__(self, outcomes=["succeeded"])
        self.string = string
        self.pub=rospy.Publisher("tts_data", String, queue_size=100)
        pass

    def execute(self, userdata):
        #test
        self.pub.publish(self.string)
        sleep(2)
        return "succeeded"

class FindObject(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','again'])
        self.xm_findobject = rospy.ServiceProxy('Find_Object', xm_ObjectDetect)
        self.angle_srv= rospy.ServiceProxy('xm_robot/simple_head_controller/look',xm_Look)
        self.base_service_client = rospy.ServiceProxy('move', xm_Move)
        self.pub=rospy.Publisher("tts_data", String, queue_size=100)
        self.tf_listener = tf.TransformListener()

    def execute(self, ud):
        global object_name,object_pos
        goal = Point()
        angle = [0.5]
        for i in range(10):
            for j in angle:
                goal.y = j
                self.angle_srv.call(goal)
                sleep(1)
                try:
                    self.object_info = self.xm_findobject.call()
                except:
                    return 'aborted'
                if len(self.object_info.objects) > 0 :
                    object_pos = self.object_info.objects
                    print object_pos
                    return 'succeeded'
        return 'aborted'        

class ArmCmd(State):
    def __init__(self,mode=0):
        State.__init__(self, outcomes=[ 'succeeded', 'aborted'])
        self.xm_pick_place_client = rospy.ServiceProxy('xm_pick_or_place', xm_PickOrPlace)
        self.base_service_client = rospy.ServiceProxy('move', xm_Move)
        self.tf_listener = tf.TransformListener()
        self.pub=rospy.Publisher("tts_data", String, queue_size=100)
        self.mode = mode
        
    def execute(self, ud): 
        rospy.loginfo('Moving arm')
        global object_pos, flag
        object_pos[flag].pos.header.stamp = rospy.Time()
        ps = self.tf_listener.transformPoint('arm_base', object_pos[flag].pos)
        ps.point.z = 0.03
        if self.mode == 0 :
            return self.pick(ps)
        elif self.mode == 1 :
            ps.point.x += 0.03
            ps.point.z += 0.02
            return self.place(ps)
            
    def pick(self, goal):
        global object_pos,flag
        rospy.loginfo('Pick object: ' + str(goal))
        self.pub.publish('i find ' + object_pos[flag].name)
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
        global flag
        rospy.loginfo('Place object: ' + str(goal))
        g = xm_PickOrPlaceRequest()
        g.action = 2
        g.goal_position = goal
        rs = self.xm_pick_place_client.call(g)
        flag += 1
        if rs.result == True:
            return 'succeeded'
        else:
            return 'aborted'         


class main():
    def __init__(self):
            rospy.init_node("gpsr")
            rospy.on_shutdown(self.shutdown)
            global object_name , object_pos
            self.sm_find= StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','again','move'])########################
            with self.sm_find:
                StateMachine.add('READY',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 0),response_cb = self.put_cb),transitions={'succeeded':'FIND_OBJECT','aborted':'READY'})
                StateMachine.add('FIND_OBJECT',FindObject(),transitions={'succeeded':'','aborted':'SPEAK'})
                StateMachine.add('SPEAK',Speak(string = 'i can not find it'),transitions={'succeeded':''})

            self.sm_grasp= StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','again','move'])########################
            with self.sm_grasp:
                StateMachine.add('GRASP',ArmCmd(mode=0),transitions={'succeeded':'PLACE','aborted':'PLACE'})
                StateMachine.add('PLACE',ArmCmd(mode=1),transitions={'succeeded':'','aborted':''})

            
            self.sm_find.execute()
            for  i in range(len(object_pos)):
                self.sm_grasp.execute()

    def put_cb(self,UserData,response):
        if response.result ==True:
            return 'succeeded'
        else :
            return 'aborted'     

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)        
        pass      
    
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass


