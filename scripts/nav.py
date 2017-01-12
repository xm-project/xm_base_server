#!/usr/bin/env python 
#encoding:utf8 

"""
author 
    
        yxm  created in 2016-07
"""

#GPSR 0x06
# export ROS_MASTER_URI='http://xm-desktop:11311'
from target import target
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
from math import atan2,cos,sin,sqrt

object_pos = PointStamped()
object_name = ''
object_flag = 0
object_try = 0
DIS = 0.07

class Go(State):
    def __init__(self,mode=0):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # Wait up to 60 seconds for the action server to become available  
        self.mode=mode
        
    def execute(self, ud):
        global target,ACTION,CURRENT_TASK,GRASP,people_af,people_bf,people_flag
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        room = [0x20,0x21,0x22,0x23,0x30,0x35,0x40,0x45]
        for i in room:
            print target[i]['name']
            print target[i]['pos']
            goal.target_pose.pose = target[i]['pos']
            self.move_base.send_goal(goal)
            self.move_base.wait_for_result(rospy.Duration(25))
            sleep(2)
        return 'succeeded'   

class main():
    def __init__(self):
            rospy.init_node("gpsr")
            rospy.on_shutdown(self.shutdown)
            global DIS,object_name

            self.sm_grasp= StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','again','move'])########################
            with self.sm_grasp:
                StateMachine.add('NAV_POSITION',Go(mode=0),transitions={'succeeded':'','aborted':'NAV_POSITION','preempted':''})
                
            self.sm_grasp.execute()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)        
        pass      
    
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass


