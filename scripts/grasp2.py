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

class FindObject(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','again'])
        self.xm_findobject = rospy.ServiceProxy('Find_Object', xm_ObjectDetect)
        self.angle_srv= rospy.ServiceProxy('xm_robot/simple_head_controller/look',xm_Look)
        self.base_service_client = rospy.ServiceProxy('move', xm_Move)
        self.tf_listener = tf.TransformListener()

    def execute(self, ud):
        global object_name,object_pos
        angle=[0.2,0.4]
        for i in range(5):
            for j in angle :
                goal = Point()
                goal.y = j
                self.angle_srv.call(goal)
                sleep(1)
                for k in range(2) :
                    self.object_info = self.xm_findobject.call()
                    if len(self.object_info.objects) > 0 :
                        object_pos = self.object_info.objects[0].pos
                        object_name = self.object_info.objects[0].name
                        print object_name
                        print object_pos
                        object_pos.point.z += 0.05
                        object_pos.header.stamp = rospy.Time()
                        object_pos.header.frame_id = 'camera_link'
                        ps = self.tf_listener.transformPoint('arm_base', object_pos)
                        print ps
                        dis = sqrt(ps.point.x*ps.point.x + ps.point.y * ps.point.y + ps.point.z * ps.point.z)
                        print dis.real
                        if dis.real<0.7 or 1.0<dis.real :
                            print 'hhhhhhh'
                            ps.point.x -= 0.8
                            ps.point.y *= 0.9
                            rs = self.base_service_client.call(ps.point.x, 0, 0)
                            rospy.sleep(2.0)
                            rs_ = self.base_service_client.call(0, ps.point.y,0)
                            rospy.sleep(2.0)
                            return 'again'
                        return 'succeeded'
            self.base_service_client.call(0.02,0,0)
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
                    if ps.point.y >= 0.15:
                        self.base_service_client.call(0,ps.point.y,0)
                        sleep(1)
                        ps.point.y = 0
                        self.object_1 = self.xm_findobject.call()
                        object_pos = self.object_1.objects[0].pos
                    rospy.sleep(2.0)
                    return 'succeeded'
            if y >= 0.03 :
                y = - 2*y
            else :
                y += 0.03
            self.base_service_client.call(0,y,0)
        return 'aborted'
        
class Speak(State):
    def __init__(self,string="",mode=0):
        State.__init__(self, outcomes=["succeeded"])
        self.string=string
        self.mode=mode
        self.pub=rospy.Publisher("tts_data", String, queue_size=100)
    def execute(self, userdata):
        self.pub.publish(self.string)
        rospy.sleep(2)
        return "succeeded"       

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

class main():
    def __init__(self):
            rospy.init_node("gpsr")
            rospy.on_shutdown(self.shutdown)
            global DIS,object_name

            self.sm_grasp= StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','again','move'])########################
            with self.sm_grasp:
                StateMachine.add('FIND_OBJECT',FindObject(),
                                 transitions={'succeeded':'TALK','again':'FIND_AGAIN','aborted':''})
                StateMachine.add('FIND_AGAIN',FindAgain(),
                                 transitions={'succeeded':'TALK','aborted':''})
                StateMachine.add('TALK',Speak('i find'+object_name),transitions={'succeeded':'GRASP'})
                StateMachine.add('GRASP',ArmCmd(mode=0),transitions={'succeeded':'PLACE','aborted':''})
                StateMachine.add('PLACE',ArmCmd(mode=1),transitions={'succeeded':'','aborted':''})
            
            self.sm_grasp.execute()

    def move_cb(self,userdata,response):
        if response.arrived == True:
            return 'succeeded'
        else:
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


