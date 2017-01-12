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

class Find_Object(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', "aborted", 'preempted'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        rospy.wait_for_service('Find_Object')
        try:
            self.ser=rospy.ServiceProxy("Find_Object",xm_ObjectDetect)
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
            self.ser=rospy.ServiceProxy("Find_Object",xm_ObjectDetect)

        result=self.ser.call()
        if result.object.pos is not None :
            #tmp_pos=result.objects.pos
            tmp_pos=result.objects.pos
            tmp_pos.header.stamp = rospy.Time()
	    tmp_pos.header.frame_id = 'camera_link'
            self.pose = self.tf_listener.transformPoint('base_link', tmp_pos)
            print tmp_pos
            print self.pose

            rospy.sleep(0.5)
            return "succeeded"      

class main():
    def __init__(self):
        rospy.init_node("whoiswho")
        self.tf_listener = tf.TransformListener()
        
        self.sm_object=StateMachine(outcomes=['succeeded','aborted','invalid','valid','preempted'])
        with self.sm_object :
            StateMachine.add('object',Find_Object(),transitions={'succeeded':'','aborted':'object'})

        out=self.sm_object.execute()
            

if __name__ == '__main__':
#     try:
    main()
