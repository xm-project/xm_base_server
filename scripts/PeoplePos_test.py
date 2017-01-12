#!/usr/bin/env python 
# encoding:utf8

"""
author

            yxm
            created in 2016-08
            
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
from std_msgs.msg import String,Int32
import threading
from xm_msgs.srv import *
from xm_msgs.msg import *
from collections import OrderedDict
import subprocess
import PyKDL
from math import atan2
from smach.state import State
from smach.state_machine import StateMachine
from smach_ros.simple_action_state import SimpleActionState

POSITION=[]
PEOPLE_IN_SIGHT=0
PERSON_ID=0      #位置编号

class Wait(State):
    def __init__(self,rec=0):
        State.__init__(self, outcomes=["succeeded"])
        self.rec=rec
        pass

    def execute(self, userdata):
        #test
        rospy.sleep(self.rec)
        return "succeeded"

class main():
    def __init__(self):
        rospy.init_node("whoiswho")
        self.tf_listener = tf.TransformListener()
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        global PERSON_ID
        
        self.sm_people=StateMachine(outcomes=['succeeded','aborted','invalid','valid','preempted'])
        with self.sm_people :
	    StateMachine.add('WAIT',Wait(rec=3),transitions={'succeeded':'PEOPLE_AMOUNT'})
            StateMachine.add('PEOPLE_AMOUNT',MonitorState("face_detector/people_tracker_measurements_array",PositionMeasurementArray,self.check_people_cb),
                                                   transitions={'valid':'PEOPLE_AMOUNT','invalid':''})
        out=self.sm_people.execute()

            
            
    def check_people_cb(self,userdata,msg):
        global POSITION,PEOPLE_IN_SIGHT
        POSITION = []
        if msg.people is not None:
            rospy.loginfo(msg.people)
         #   print len(msg.people)            
            PEOPLE_IN_SIGHT=len(msg.people)
            self.tmp_pos=msg.people                
            pose = PointStamped()
            pose.header = msg.header
	    pose.header.stamp = rospy.Time()
            for i in range(PEOPLE_IN_SIGHT):
                pose.point= self.tmp_pos[i].pos
                pointx = self.tf_listener.transformPoint('base_link', pose)
                POSITION.append(pointx)
            rospy.logwarn(POSITION)
            return False
        else :
            return True


if __name__ == '__main__':
#     try:
    main()
