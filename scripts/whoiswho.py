#!/usr/bin/env python 
# encoding:utf8

"""
author

            yxm
            created in 2016-08
            
"""
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
from actionlib_msgs.msg import GoalStatus
from time import sleep
from std_msgs.msg import String,Int32,Bool
import threading
from xm_msgs.srv import *
from xm_msgs.msg import *
import subprocess
from math import *
from copy import deepcopy
#status=0x08

object_name = ''
object_pos = PointStamped()
person_id=0      #位置编号
person_name = ''

POSITION_TMP = []
POSITION_BF=[]
POSITION_AF=[]
PEOPLE_TMP = 0
PEOPLE_IN_SIGHT=0

NAME = []
OBJECT = []
        
class Speak(State):
    def __init__(self,string="",mode=0):
        State.__init__(self, outcomes=["succeeded"])
        self.string=string
        self.mode=mode
        self.pub=rospy.Publisher("tts_data", String, queue_size=100)
        
    def execute(self, userdata):
        global NAME,OBJECT,person_name,object_name
        if self.mode == 1:
            for i in range(len(NAME)):
                if person_name == NAME[i]:
                    self.num = i 
            self.object = OBJECT[self.num]
            self.string = 'Your name is ' + str(person_name) + ',you need a ' + str(self.object)
        elif self.mode ==2 :
            self.string = 'i find a ' + object_name
        rospy.logwarn(self.string)
        self.pub.publish(self.string)
        rospy.sleep(2)
        return "succeeded"      

class Wait(State):
    def __init__(self,rec=0):
        State.__init__(self, outcomes=["succeeded"])
        self.rec=rec
        pass

    def execute(self, userdata):
        rospy.sleep(self.rec)
        return "succeeded"

class Move(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded","aborted","preempted"])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
    def execute(self, ud):
        global POSITION_BF,POSITION_AF,person_id
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        bx=POSITION_BF[person_id].point.x
        by=POSITION_BF[person_id].point.y
        ax=POSITION_AF[person_id].point.x
        ay=POSITION_AF[person_id].point.y
        angle = atan2((by-ay),(bx-ax))
        q_angle = quaternion_from_euler(0, 0,angle)
        self.q=Quaternion(*q_angle)
        goal.target_pose.pose=(Pose(Point(ax,ay,0),self.q))
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result(rospy.Duration(25))
        if self.move_base.get_goal_status_text() == 'Goal reached.' :
            return 'succeeded'
        else:
            return 'aborted'
        
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
        sleep(3)
        return 'succeeded'
        
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
                    try:
                        self.object_info = self.xm_findobject.call()
                    except:
                        return 'aborted'
                    if len(self.object_info.objects) > 0 :
                        object_name = self.object_info.objects[0].name
                        object_pos = self.object_info.objects[0].pos
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
            self.base_service_client.call(0.02,0,0)
            sleep(2.0)
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
        goal.y = 0.3
        self.angle_srv.call(goal)
        sleep(1)
        y = 0.00
        for i in range(5) :
            for j in range(2) :
                self.object_info = self.xm_findobject.call()
                if len(self.object_info.objects) > 0 :
                    object_name = self.object_info.objects[0].name
                    object_pos = self.object_info.objects[0].pos
                    object_pos.header.frame_id = 'camera_link'
                    object_pos.header.stamp = rospy.Time()
                    ps = self.tf_listener.transformPoint('arm_base', object_pos)
                    if ps.point.y >=0.15 :
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
   
class FaceSrv(State):
    def __init__(self):
        State.__init__(self, outcomes=[ 'succeeded', 'aborted'])
        self.face_srv = rospy.ServiceProxy("Find_Face", xm_FaceRecognize)
        
    def execute(self, userdata):       
        global person_name
        goal = xm_FaceRecognizeRequest()
        goal.command = 'remember'
        goal.name = person_name
        rs = self.face_srv.call(goal)
        if rs.result ==True:
            return 'succeeded'  
        else :
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
        sleep(1)
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
        rospy.init_node("whoiswho")
        rospy.on_shutdown(self.shutdown)
        self.tf_listener = tf.TransformListener()
        global person_id,NAME,object_name,person_name

        self.waypoints=[]

        Location= (Point(4.8,1.7,0),  #找人数的点
                   Point(9.770, 3.939, 0.000),   #出门抓东西的点
                   Point(10.839,9.5,0))     #结束出门的点
        quaternions=[]

        euler_angles=[0,1.57,1.934];
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes="sxyz")
            q = Quaternion(*q_angle)
            quaternions.append(q)
        for i in range(3):
            self.waypoints.append(Pose(Location[i], quaternions[i]))

        nav_states={}
        for room in range(3):
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = "map"
            nav_goal.target_pose.header.stamp = rospy.Time.now()
            nav_goal.target_pose.pose = self.waypoints[room]
            move_base_state = SimpleActionState("move_base", MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb, 
                                                exec_timeout=rospy.Duration(120.0),
                                                server_wait_timeout=rospy.Duration(120.0))
            nav_states[room] = move_base_state   
        
        self.sm_amount=StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid'])
        with self.sm_amount :
            StateMachine.add('PUT',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 3),response_cb = self.put_cb),transitions={'succeeded':'WAIT1','aborted':'PUT'})
            StateMachine.add('WAIT1',Wait(rec=15),transitions={'succeeded':'DOOR'})
            StateMachine.add('DOOR',MonitorState('DoorState',Bool,self.door_cb),transitions={'invalid':'NAV1','valid':'DOOR'})
            StateMachine.add('NAV1',nav_states[0],transitions={'succeeded':'TURN_CAMERA','aborted':'NAV1'})
            StateMachine.add('TURN_CAMERA',TurnCamera(angle = pi/8),transitions={'succeeded':'SPEAK'})
            StateMachine.add('SPEAK',Speak(string='Please look at me'),transitions={'succeeded':'WAIT'})
            StateMachine.add('WAIT',Wait(rec=1),transitions={'succeeded':'PEOPLE_AMOUNT'})
            StateMachine.add('PEOPLE_AMOUNT',ServiceState("Find_Face",xm_FaceRecognize,xm_FaceRecognizeRequest(command='detect'),response_cb = self.check_people_cb1),
                                                   transitions={'aborted':'PEOPLE_AMOUNT','succeeded':'TURN_CAMERA2'})
            StateMachine.add('TURN_CAMERA2',TurnCamera(angle = -pi/8),transitions={'succeeded':'SPEAK2'})
            StateMachine.add('SPEAK2',Speak(string='Please look at me'),transitions={'succeeded':'WAIT2'})
            StateMachine.add('WAIT2',Wait(rec=1),transitions={'succeeded':'PEOPLE_AMOUNT2'})
            StateMachine.add('PEOPLE_AMOUNT2',ServiceState("Find_Face",xm_FaceRecognize,xm_FaceRecognizeRequest(command='detect'),response_cb = self.check_people_cb2),
                                                   transitions={'aborted':'PEOPLE_AMOUNT2','succeeded':'TURN_CAMERA3'})
            StateMachine.add('TURN_CAMERA3',TurnCamera(angle = 0),transitions={'succeeded':''})

        self.sm_recognize=StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid'])
        with self.sm_recognize :
            StateMachine.add('MOVE',Move(),transitions={'succeeded':'ASK_NAME','aborted':'ASK_NAME','preempted':''})
            StateMachine.add('ASK_NAME',Speak(string="what is your name and what do you need"),transitions={'succeeded':'WAIT4NAME'})
            StateMachine.add('WAIT4NAME',MonitorState("task_coming", xm_Task, self.name_cb),transitions={'invalid':'LOOK','valid':'WAIT4NAME'})
            StateMachine.add('LOOK',Speak(string="please look at me"),transitions={'succeeded':'WAIT'})
            StateMachine.add('WAIT',Wait(rec=1),transitions={'succeeded':'RECOGNIZE'})
            StateMachine.add('RECOGNIZE', FaceSrv(),transitions={"succeeded":"OKAY","aborted":"RECOGNIZE",})
            StateMachine.add('OKAY',Speak(string="okay,i remember you"),transitions={'succeeded':''})
            
        self.sm_out_back=StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','again'])
        with self.sm_out_back :
            StateMachine.add('GET_OUT',nav_states[1],transitions={'succeeded':'GRASP','aborted':'GET_OUT'})
            self.sm_grasp= StateMachine(outcomes=['succeeded','aborted','preempted','valid','invalid','again'])########################
            with self.sm_grasp:
                StateMachine.add('READY',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 0),response_cb = self.put_cb),transitions={'succeeded':'FIND_OBJECT','aborted':'READY'})
                StateMachine.add('FIND_OBJECT',FindObject(),transitions={'succeeded':'TALK','again':'FIND_AGAIN','aborted':'PUT'})
                StateMachine.add('FIND_AGAIN',FindAgain(),transitions={'succeeded':'TALK','aborted':'PUT'})
                StateMachine.add('TALK',Speak(mode = 2),transitions={'succeeded':'GRASP'})
                StateMachine.add('GRASP',ArmCmd(mode=0),transitions={'succeeded':'PUT','aborted':'PUT'})
                StateMachine.add('PUT',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 3),response_cb = self.put_cb),transitions={'succeeded':'','aborted':'PUT'})
           
            StateMachine.add('GRASP',self.sm_grasp,transitions={'succeeded':'GET_BACK'})
            StateMachine.add('GET_BACK',nav_states[0],transitions={'succeeded':'TURN_CAMERA'})
            StateMachine.add('TURN_CAMERA',TurnCamera(angle = pi/8),transitions={'succeeded':'SPEAK'})
            StateMachine.add('SPEAK',Speak(string='Please look at me'),transitions={'succeeded':'WAIT'})
            StateMachine.add('WAIT',Wait(rec=1),transitions={'succeeded':'PEOPLE_AMOUNT'})
            StateMachine.add('PEOPLE_AMOUNT',ServiceState("Find_Face",xm_FaceRecognize,xm_FaceRecognizeRequest(command='detect'),response_cb = self.check_people_cb1),
                                                   transitions={'aborted':'PEOPLE_AMOUNT','succeeded':'TURN_CAMERA2'})
            StateMachine.add('TURN_CAMERA2',TurnCamera(angle = -pi/8),transitions={'succeeded':'SPEAK2'})
            StateMachine.add('SPEAK2',Speak(string='Please look at me'),transitions={'succeeded':'WAIT2'})
            StateMachine.add('WAIT2',Wait(rec=1),transitions={'succeeded':'PEOPLE_AMOUNT2'})
            StateMachine.add('PEOPLE_AMOUNT2',ServiceState("Find_Face",xm_FaceRecognize,xm_FaceRecognizeRequest(command='detect'),response_cb = self.check_people_cb2),
                                                   transitions={'aborted':'PEOPLE_AMOUNT2','succeeded':'TURN_CAMERA3'})
            StateMachine.add('TURN_CAMERA3',TurnCamera(angle = 0),transitions={'succeeded':''})

        self.sm_identify=StateMachine(outcomes=['succeeded','aborted','preempted','person'])
        with self.sm_identify :
            StateMachine.add('MOVE',Move(),transitions={'succeeded':'LOOK','aborted':'LOOK','preempted':'MOVE'})
            StateMachine.add('LOOK',Speak(string="please look at me"),transitions={'succeeded':'WAIT2'})
            StateMachine.add('WAIT2',Wait(rec=1),transitions={'succeeded':'IDENTIFY'})
            StateMachine.add('IDENTIFY', ServiceState("Find_Face", xm_FaceRecognize, xm_FaceRecognizeRequest(command='recognize'),
                                                      response_cb=self.identify_people_cb,outcomes=['succeeded','aborted','person']),
                             transitions={'succeeded':'OKAY','aborted':'IDENTIFY','person':'PERSON'})
            StateMachine.add('PERSON',Speak(mode=1),transitions={'succeeded':'SPEAK_PLACE'})
            StateMachine.add('SPEAK_PLACE',Speak(string='Please hold your stuff'),transitions={'succeeded':'PLACE'})
            StateMachine.add('PLACE',ArmCmd(mode=1),transitions={'succeeded':'PUT','aborted':'PUT'})                          
            StateMachine.add('PUT',ServiceState('xm_pick_or_place',xm_PickOrPlace,xm_PickOrPlaceRequest(action = 3),response_cb = self.put_cb),transitions={'succeeded':'','aborted':'PUT'})
            StateMachine.add('OKAY',Speak(mode=1),transitions={'succeeded':''})
            
        self.sm_end=StateMachine(outcomes=['succeeded','aborted','preempted'])
        with self.sm_end :
            StateMachine.add('NAV_OUT',nav_states[2],transitions={'succeeded':'','aborted':'NAV_OUT'})
            
        self.sm_amount.execute()    
        for i in xrange(PEOPLE_IN_SIGHT):
            person_id = i
            out=self.sm_recognize.execute()
            
        self.sm_out_back.execute()    
        for i in xrange(PEOPLE_IN_SIGHT):
            person_id = i
            out=self.sm_identify.execute()
        self.sm_end.execute()    

        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)        
        pass      

    def move_base_result_cb(self, userdata, status, result) :
        if status == actionlib.GoalStatus.SUCCEEDED :
            return 'succeeded'
        return 'aborted'

    def door_cb(self,userdata,msg):
        if msg.data==True:
            subprocess.call(["rosservice","call","/move_base/clear_costmaps"])
            return False
        else:
            return True
        
    def name_cb(self,userdata,msg):
        global NAME,person_name,target,OBJECT
        if msg.place is not None :
            person_name = deepcopy(target[msg.place]['name'])
            rospy.logwarn(person_name)
            NAME.append(person_name)
            OBJECT.append(target[msg.task]['name'])
            return False
        else:
            return True

    def check_people_cb1(self,userdata,response):
        global POSITION_TMP,PEOPLE_TMP
        POSITION_TMP = []
        PEOPLE_TMP = 0
        if response.result == True:       
            self.tmp_pos = response.pos
            for i in self.tmp_pos:
                if isnan(i.pose.position.x):
                    return 'aborted'
            PEOPLE_TMP += len(response.pos)
            
            rospy.loginfo(self.tmp_pos)
            for i in range(PEOPLE_TMP):
                pose = self.tmp_pos[i]
                pose.header.stamp = rospy.Time()
                pose.header.frame_id = 'camera_link'
                self.tf_listener.waitForTransform('base_link',pose.header.frame_id,rospy.Time(0),rospy.Duration(60.0))
                pointb = self.tf_listener.transformPoint('base_link', pose)
                POSITION_TMP.append(pointb)
                rospy.logwarn(pointb)
            rospy.logwarn("people now ="+str(PEOPLE_TMP))
            sleep(3)
            return 'succeeded'
        elif response.message[0] == 'No_face':
            return 'succeeded'
        else :
            return 'aborted'
        
    def check_people_cb2(self,userdata,response):
        global POSITION_BF, POSITION_AF, POSITION_TMP, PEOPLE_IN_SIGHT, PEOPLE_TMP
        POSITION_AF = []
        POSITION_BF = []
        if response.result == True:
            self.tmp_pos = response.pos
            for i in self.tmp_pos:
                if isnan(i.pose.position.x):
                    return 'aborted'
            self.tmp_num = len(response.pos)
            PEOPLE_TMP += len(response.pos)
            
            for i in range(self.tmp_num):
                pose = self.tmp_pos[i]
                pose.header.stamp = rospy.Time()
                pose.header.frame_id = 'camera_link'
                self.tf_listener.waitForTransform('base_link',pose.header.frame_id,rospy.Time(),rospy.Duration(60.0))
                pointb = self.tf_listener.transformPoint('base_link',pose)
                POSITION_TMP.append(pointb)
                
            for i in range(PEOPLE_TMP-1,-1,-1):
                for j in range(i):
                    if (POSITION_TMP[j].point.y < POSITION_TMP[j+1].point.y):
                        POSITION_TMP[j], POSITION_TMP[j+1] = POSITION_TMP[j+1],POSITION_TMP[j]
                        
            POSITION_BF.append( POSITION_TMP[0] )
            for i in range(PEOPLE_TMP-1):
                x1 = POSITION_TMP[i].point.x
                y1 = POSITION_TMP[i].point.y
                x2 = POSITION_TMP[i+1].point.x
                y2 = POSITION_TMP[i+1].point.y
                dis = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)
                if (dis >= 0.16) :
                    POSITION_BF.append( POSITION_TMP[i+1] )
                    
            self.pos = deepcopy(POSITION_BF)
            POSITION_BF = []
            for i in self.pos:
                if (i.point.x*i.point.x + i.point.y*i.point.y <= 9) :                    #距离平方小于8算是场内？？？？？？？？？？？？？
                    POSITION_BF.append(i)
            
            PEOPLE_IN_SIGHT = len(POSITION_BF)
            for i in range(PEOPLE_IN_SIGHT):
                pose = deepcopy(POSITION_BF[i])
                angle=atan2(pose.point.y,pose.point.x)
                pose.point.x -= 0.8*cos(angle)
                pose.point.y -= 0.8*sin(angle)
                self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                pointa = self.tf_listener.transformPoint('map', pose)
                POSITION_AF.append(pointa)
                POSITION_BF[i] = self.tf_listener.transformPoint('map', POSITION_BF[i])
                rospy.logwarn(POSITION_BF[i])
            rospy.logwarn("PEOPLE_IN_SIGHT="+str(PEOPLE_IN_SIGHT))
            sleep(3)
            return 'succeeded'
        elif response.message[0] == 'No_face':
            return 'succeeded'
        else :
            return 'aborted'
        
    def identify_people_cb(self,userdata,response):
        global NAME , OBJECT , object_name , person_name
        if response.result==True:
            person_name = response.message[0]
            rospy.logwarn(person_name)
            for i in range(len(NAME)):
                if person_name == NAME[i]:
                    self.num = i 
                    if OBJECT[i] == object_name:
                        return 'person'
            return "succeeded"
        else:
            return "aborted"
             
    def put_cb(self,UserData,response):
        if response.result ==True:
            return 'succeeded'
        else :
            return 'aborted'     
             
if __name__ == '__main__':
#     try:
    main()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("SMACH test finished.")        
        
        
