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
from math import *


people_bf = PointStamped()
people_af = PointStamped()
people_flag = 0

ACTION = []
GOAL = []
TASK_NUM = 0
CURRENT_TASK = -1

object_name = ''
object_pos= PointStamped()
#task
GO = 0x01
FIND = 0x02
GRASP = 0x03
FOLLOW = 0x04
TALK = 0x05
PLACE = 0x07
#target
PERSON=0X08
SPEAKER=0X09 


            
class FindObject(State):
    def __init__(self,mode):
        State.__init__(self, outcomes=['succeeded', 'aborted','again'])
        self.xm_findobject = rospy.ServiceProxy('Find_Object', xm_ObjectDetect)
        self.angle_srv= rospy.ServiceProxy('xm_robot/simple_head_controller/look',xm_Look)
        self.base_service_client = rospy.ServiceProxy('move', xm_Move)
        self.tf_listener = tf.TransformListener()
        self.mode = mode

    def execute(self, ud):
        global object_name,object_pos,GOAL,CURRENT_TASK,target,TASK_NUM,ACTION,Go
        self.target = target[ord(GOAL[CURRENT_TASK])]
        object_name = self.target['name']
        angle = [0,0.3,0.5]
        for i in range(5):
            for j in angle :
                goal = Point()
                goal.y = j
                self.angle_srv.call(goal)
                sleep(1)
                for k in range(2) :
                    self.object_info = self.xm_findobject.call()
                    if len(self.object_info.objects) > 0 :
                        if self.mode == 0:
                            for i in self.object_info.objects :
                                if i.name == object_name:
                                    object_pos = i.pos
                        if self.mode == 1:
                            object_pos = self.object_info.objects[0].pos
                        print object_name
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
            self.base_service_client.call(0.05,0,0)##待确定!!!!!!!!!!!!!!!!!!!!!!!!!!
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
        global object_name,object_pos,GOAL,CURRENT_TASK,target,ACTION,GO,TASK_NUM
        goal = Point()
        self.target = target[ord(GOAL[CURRENT_TASK])]
        goal.y = 0.3
        for i in range(TASK_NUM):
            if ACTION[i] == GO :
                if  GOAL[i] == 0x30 or GOAL[i] == 0x45 :
                    goal.y = 0.3
                    break
                elif GOAL[i] == 0x40 :
                    goal.y = 0.1
                    break
        self.angle_srv.call(goal)
        sleep(1)
        y = 0.00
        for i in range(5) :
            for j in range(2) :
                self.object_info = self.xm_findobject.call()
                if len(self.object_info.objects) > 0 :
                    for i in self.object_info.objects :
                        if i.name == object_name:
                            object_pos = i.pos
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
        global object_pos,target,GOAL,CURRENT_TASK
        if object_pos.point == Point(0,0,0):
            return 'succeeded'
        ps = self.tf_listener.transformPoint('arm_base', object_pos)
        if self.mode == 0 :
            return self.pick(ps)
        elif self.mode == 1 :##待确定!!!!!!!!!!!!!!!!!!!!!!!!!!
#             ps.point.x = 0.9
#             ps.point.y = 0.0
#             ps.point.z =0.2
            ps.point.x += 0.05
            return self.place(ps)
            
    def pick(self, goal):
        rospy.loginfo('Pick object: ' + str(goal))
        g = xm_PickOrPlaceRequest()
        g.action = 1
        g.goal_position = goal
        rs = self.xm_pick_place_client.call(g)
        #sleep(2)
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

class Wait(State):
    def __init__(self,rec=0):
        State.__init__(self, outcomes=["succeeded"])
        self.rec=rec
        pass

    def execute(self, userdata):
        rospy.sleep(self.rec)
        return "succeeded"

class TurnCamera(State):
    def __init__(self, angle=0):
        State.__init__(self, outcomes=["succeeded"])
        self.angle=angle
        self.angle_srv= rospy.ServiceProxy('xm_robot/simple_head_controller/look',xm_Look)
        
    def execute(self, ud):
        goal = Point()
        goal.z = self.angle
        self.angle_srv.call(goal)
        self.angle_srv.wait_for_service(20)
        return 'succeeded'    

class GetTask(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','go','find','follow','grasp','talk','place'])
    def execute(self, UserData):
        global ACTION,TASK_NUM,CURRENT_TASK,GO,FIND,FOLLOW,GRASP,TALK,PLACE
        CURRENT_TASK=CURRENT_TASK+1
        if CURRENT_TASK==TASK_NUM:
            return 'succeeded'
        
        self.current_action=ord(ACTION[CURRENT_TASK])
        if self.current_action == GO:
            return 'go'
        elif self.current_action == FIND:
            return 'find'
        elif self.current_action == FOLLOW:
            return 'follow'
        elif self.current_action == GRASP:
            return 'grasp'
        elif self.current_action == TALK:
            return 'talk'
        elif self.current_action == PLACE:
            return 'place'
        else:
            CURRENT_TASK-=1
            return 'aborted'
        
class PersonOrPosition(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted','person','position'])
    def execute(self,userdata):
        global GOAL,PERSON,SPEAKER,CURRENT_TASK
        if ord(GOAL[CURRENT_TASK])==PERSON or ord(GOAL[CURRENT_TASK])==SPEAKER:
            return 'person'
        else:
            return 'position'

class Go(State):
    def __init__(self,mode=0):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # Wait up to 60 seconds for the action server to become available  
        self.mode=mode
        
    def execute(self, ud):
        global target,CURRENT_TASK,people_af,people_bf,people_flag
        if self.mode==0: 
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose = target[ord(GOAL[CURRENT_TASK])]['pos']
            self.move_base.send_goal(goal)
            self.move_base.wait_for_result(rospy.Duration(25))
        else:
            if people_flag == 1 :
                people_flag = 0
                return 'succeeded'
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            bx=people_bf.point.x
            by=people_bf.point.y
            ax=people_af.point.x
            ay=people_af.point.y
            angle = atan2((by-ay),(bx-ax))
            q_angle = quaternion_from_euler(0, 0, angle )
            self.q=Quaternion(*q_angle)
            goal.target_pose.pose=(Pose(Point(ax,ay,0),self.q))
            self.move_base.send_goal(goal)            
            self.move_base.wait_for_result(rospy.Duration(60))
#         print self.move_base.get_result()
        sleep(8)
        return 'succeeded'            
            
class Speak(State):
    def __init__(self,string="",mode=0):
        State.__init__(self, outcomes=["succeeded"])
        self.string=string
        self.mode=mode
        self.pub=rospy.Publisher("tts_data", String, queue_size=100)
    def execute(self, userdata):
        global target,GOAL,CURRENT_TASK
        if self.mode == 2:
            self.string = 'i find ' + target[ord(GOAL[CURRENT_TASK])]['name']
        self.pub.publish(self.string)
        rospy.sleep(2)
        return "succeeded"               
            
class GPSR():
    def __init__(self):
        rospy.init_node("gpsr")
        rospy.on_shutdown(self.shutdown)
        self.tf_listener=tf.TransformListener()
        global object_name
        self.state=None
        self.waypoints=[]
        #需要一个进门地点（在此点询问命令）##待确定!!!!!!!!!!!!!!!!!!!!!!!!!!
        Location= (Point(4.817, 1.928, 0.000),
                            Point(2,0.5,0) )#备用出去的点
        quaternions=[]##待确定!!!!!!!!!!!!!!!!!!!!!!!!!!
        euler_angles=[0,0];##待确定!!!!!!!!!!!!!!!!!!!!!!!!!!
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes="sxyz")
            q = Quaternion(*q_angle)
            quaternions.append(q)
        for i in range(2):
            self.waypoints.append(Pose(Location[i], quaternions[i]))
        point_locations = (("Point1", self.waypoints[0]),              #第一个点
                                        ("Point2", self.waypoints[1]))                          
        self.room_locations = OrderedDict(point_locations)
        nav_states={}
        self.people_in_sight=0
        for room in self.room_locations.iterkeys():
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = "map"
            nav_goal.target_pose.header.stamp = rospy.Time.now()
            nav_goal.target_pose.pose = self.room_locations[room]
            move_base_state = SimpleActionState("move_base", MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb, 
                                                exec_timeout=rospy.Duration(120.0),
                                                server_wait_timeout=rospy.Duration(120.0))
            nav_states[room] = move_base_state
    
        sm_nav1= StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid'])
        with sm_nav1:
            StateMachine.add('PUT',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 3),response_cb = self.put_cb),transitions={'succeeded':'DOOR','aborted':'PUT'})
            StateMachine.add('DOOR',MonitorState('DoorState',Bool,self.door_cb),transitions={'invalid':'WAIT','valid':'DOOR'})
            StateMachine.add('WAIT',Wait(rec=2),transitions={'succeeded':'NAV_1'})
            StateMachine.add('NAV_1',nav_states["Point1"],transitions={'succeeded':'NAV_1_AGAIN','aborted':'NAV_1_AGAIN','preempted':'NAV_1_AGAIN'})
            StateMachine.add('NAV_1_AGAIN',nav_states["Point1"],transitions={'succeeded':'SPEAK1','aborted':'SPEAK1','preempted':'SPEAK1'})
            StateMachine.add('SPEAK1', Speak(string='I arrived,please ask me a question'), transitions={'succeeded':''})   

        sm_nav2=StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','go','find','grasp','follow','talk','person','position','again','place','turn'])
        with sm_nav2:
            StateMachine.add('RECEIVE_NAV2_CMD',MonitorState("task_coming2",xm_TaskArray,self.nav2_cmd_cb),
                             transitions={'invalid':'GET_NEXT_TASK','valid':'RECEIVE_NAV2_CMD'})
            StateMachine.add('GET_NEXT_TASK',GetTask(),
                             transitions={'succeeded':'', 'find':'FIND', 'go': 'GO', 'follow': 'FOLLOW', 'grasp': 'GRASP', 'talk': 'TALK', 'place': 'PLACE', 'aborted': 'GET_NEXT_TASK'})
             
            self.sm_find=StateMachine(outcomes=['succeeded','aborted','preempted','person','position','valid','invalid','turn'])
            with self.sm_find:
                self.sm_person=StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','turn'])
                with self.sm_person:
                    StateMachine.add('TURN1',TurnCamera(angle = pi/8),transitions={'succeeded':'GET_PEOPLE_POSITION1'})##待确定!!!!!!!!!!!!!!!!!!!!!!!!!!
                    StateMachine.add('GET_PEOPLE_POSITION1',ServiceState('Find_Face',xm_FaceRecognize,xm_FaceRecognizeRequest(command='detect'),
                                                                         response_cb=self.people_position_cb,outcomes=['succeeded','turn','aborted']),
                                 transitions={'succeeded':'TURN3','turn':'TURN2','aborted':'GET_PEOPLE_POSITION1'})
                    StateMachine.add('TURN2',TurnCamera(angle = -pi/8),transitions={'succeeded':'GET_PEOPLE_POSITION2'})
                    StateMachine.add('GET_PEOPLE_POSITION2',ServiceState('Find_Face',xm_FaceRecognize,xm_FaceRecognizeRequest(command='detect'),
                                                                         response_cb=self.people_position_cb,outcomes=['succeeded','aborted','turn']),
                                 transitions={'succeeded':'TURN3','aborted':'GET_PEOPLE_POSITION2','turn':'TURN3'})
                    StateMachine.add('TURN3',TurnCamera(angle = 0),transitions={'succeeded':'NAV2_PEOPLE'})
                    StateMachine.add('NAV2_PEOPLE', Go(mode=1),transitions={'succeeded':'SPEAK'})  
                    StateMachine.add('SPEAK',Speak(string="I find you"),transitions={'succeeded':''})

                self.sm_position=StateMachine(outcomes=['succeeded','aborted','preempted'])#######################
                with self.sm_position:
                    StateMachine.add('NAV_POSITION',Go(mode=0),transitions={'succeeded':'SPEAK','aborted':'NAV_POSITION','preempted':''})
                    StateMachine.add('SPEAK',Speak(string = "i find it"),transitions={'succeeded':''})
                     
                StateMachine.add('PERSON_OR_POS',PersonOrPosition(),transitions={'person':'PERSON','position':'POSITION'})   
                StateMachine.add('PERSON',self.sm_person,transitions={'succeeded':''})
                StateMachine.add('POSITION',self.sm_position,transitions={'succeeded':''})
                 
            self.sm_go=StateMachine(outcomes=['succeeded','aborted','preempted'])###########################
            with self.sm_go:
                StateMachine.add('NAV_GO',Go(mode=0),transitions={'succeeded':'SPEAK','aborted':'NAV_GO','preempted':''})
                StateMachine.add('SPEAK',Speak(string = "i arrived"),transitions={'succeeded':''})
             
            self.sm_follow=StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','turn'])#######################
            with self.sm_follow:
#                 StateMachine.add('TURN',TurnCamera(angle=0),transitions={'succeeded':'GET_PEOPLE_POSITION'})
#                 StateMachine.add('GET_PEOPLE_POSITION',ServiceState('Find_Face',xm_FaceRecognize,xm_FaceRecognizeRequest(command='detect'),
#                                                                     response_cb=self.people_position_cb,outcomes=['succeeded','turn','aborted']),
#                                  transitions={'succeeded':'NAV2_PEOPLE','aborted':'GET_PEOPLE_POSITION'})
#                 StateMachine.add('NAV2_PEOPLE', Go(mode=1),transitions={'succeeded':'SPEAK'})  
                StateMachine.add('SPEAK',Speak(string="i will follow you"),transitions={'succeeded':'FOLLOW'})
                StateMachine.add('FOLLOW',ServiceState('follow' , Trigger,
                                                        response_cb=self.base_cb, outcomes=['succeeded' , 'failed']) , 
                                                        transitions={'succeeded':'WAIT','failed':'FOLLOW'})
                StateMachine.add('WAIT',Wait(rec=2),transitions={'succeeded':'STOP_OR_NOT'})
                StateMachine.add("STOP_OR_NOT",MonitorState('people', xm_People, self.stop_cb),transitions={'invalid':'STOP','valid':'STOP_OR_NOT' })
                StateMachine.add('STOP',ServiceState('follow' , Trigger,
                                                            response_cb=self.base_cb, outcomes=['succeeded' , 'failed']) , 
                                                            transitions={'succeeded':'','failed':'STOP'})

            self.sm_grasp= StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','again'])########################
            with self.sm_grasp:
              #  StateMachine.add('NAV_POSITION',Go(mode=0),transitions={'succeeded':'READY','aborted':'NAV_POSITION','preempted':''})
                StateMachine.add('READY',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 0),response_cb = self.put_cb),transitions={'succeeded':'FIND_OBJECT','aborted':'READY'})
                StateMachine.add('FIND_OBJECT',FindObject(mode=0),transitions={'succeeded':'TALK','again':'FIND_AGAIN','aborted':'FIND_ANY'})
                StateMachine.add('FIND_AGAIN',FindAgain(),transitions={'succeeded':'TALK','aborted':'PUT'})
                StateMachine.add('FIND_ANY',FindObject(mode=1),transitions={'succeeded':'TALK','aborted':'PUT','again':'FIND_AGAIN'})
                StateMachine.add('TALK',Speak(mode = 2),transitions={'succeeded':'GRASP'})
                StateMachine.add('GRASP',ArmCmd(mode=0),transitions={'succeeded':'PUT','aborted':'PUT'})
                StateMachine.add('PUT',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 3),response_cb = self.put_cb),transitions={'succeeded':'','aborted':'PUT'})
                
            self.sm_talk= StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid'])###############################
            with self.sm_talk:
                StateMachine.add('SPEAK',Speak(string="talk"),transitions={'succeeded':''})
                
            self.sm_place= StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid'])###############################
            with self.sm_place :
                StateMachine.add('PLACE',ArmCmd(mode=1),transitions={'succeeded':'PUT','aborted':''})
                StateMachine.add('PUT',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 3),response_cb = self.put_cb),transitions={'succeeded':'','aborted':'PUT'})
                
            StateMachine.add('GO',self.sm_go,transitions={'succeeded':'GET_NEXT_TASK','aborted':'GO'})
            StateMachine.add('FIND',self.sm_find,transitions={'succeeded':'GET_NEXT_TASK','aborted':'FIND'})
            StateMachine.add('FOLLOW',self.sm_follow,transitions={'succeeded':'GET_NEXT_TASK','aborted':'FOLLOW'})
            StateMachine.add('GRASP',self.sm_grasp,transitions={'succeeded':'GET_NEXT_TASK','aborted':'GRASP'})
            StateMachine.add('TALK',self.sm_talk,transitions={'succeeded':'GET_NEXT_TASK','aborted':'TALK'})
            StateMachine.add('PLACE',self.sm_place,transitions={'succeeded':'GET_NEXT_TASK','aborted':'PLACE'})            
             
        out=sm_nav1.execute()
        out=sm_nav2.execute()
         
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)        
        pass      
    
    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            pass
        
    def door_cb(self,userdata,msg):
        if msg.data==True:
            subprocess.call(["rosservice","call","/move_base/clear_costmaps"])
            return False
        else:
            return True

    def start_cb(self,userdata,msg):
        if msg.task==0x01 :
            pub=rospy.Publisher("tts_data", String, queue_size=100)
            pub.publish("yes, my lord")
            return False
        else :
            return True
        
    def nav2_cmd_cb(self,userdata,msg):
        global ACTION,GOAL,TASK_NUM,CURRENT_TASK
        if(msg is not None):
            TASK_NUM=len(msg.target)
            self.msg=msg
            ACTION=self.msg.action
            GOAL=self.msg.target
            CURRENT_TASK=-1
            print(TASK_NUM)
            print(ACTION)
            print(GOAL)
            return False
        else:
            return True
        
    def people_position_cb(self,userdata,response):
        global people_bf,people_af,people_flag
        if response.result == True :
            self.tmp_pos = response.pos[0]
#             rospy.logwarn(self.tmp_pos)
            self.tmp_pos.header.frame_id = 'camera_link'
            self.tmp_pos.header.stamp = rospy.Time()
            self.tf_listener.waitForTransform('map',self.tmp_pos.header.frame_id,rospy.Time(0),rospy.Duration(60.0))
            people_bf = self.tf_listener.transformPoint('map', self.tmp_pos)
            self.tf_listener.waitForTransform('base_link',self.tmp_pos.header.frame_id,rospy.Time(0),rospy.Duration(60.0))
            self.tmp_pos = self.tf_listener.transformPoint('base_link', self.tmp_pos)
            angle = atan2(self.tmp_pos.point.y,self.tmp_pos.point.x)
            self.tmp_pos.point.x -= 0.8*cos(angle)##待确定!!!!!!!!!!!!!!!!!!!!!!!!!!
            self.tmp_pos.point.y -= 0.8*sin(angle)
            self.tf_listener.waitForTransform('map','base_link',rospy.Time(0),rospy.Duration(60.0))
            people_af = self.tf_listener.transformPoint('map', self.tmp_pos)
            rospy.logwarn(people_bf)
            people_flag = 0
            return 'succeeded'
        elif response.message[9] == 'No_face':
            if people_flag == 0:
                people_flag = 1
            return 'turn'
        else:
            return 'aborted'       

    def base_cb(self, userdata, response) :
        if response.success==True :
            return 'succeeded'
        else :
            return 'failed'
        
    def stop_cb(self,userdata,msg):
        for i in msg.person:
            if i.pose.data == 'raising' :
                return False
        return True        
            
    def move_cb(self,userdata,response):
        if response.arrived == True:
            return 'succeeded'
        else:
            return 'aborted'
        
    def put_cb(self,UserData,response):
        if response.result ==True:
            return 'succeeded'
        else :
            return 'aborted'
            
if __name__ == "__main__":
    try:
        GPSR()
    except KeyboardInterrupt:
        pass
