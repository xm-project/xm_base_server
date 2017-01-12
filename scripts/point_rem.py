#!/usr/bin/env python 
#encoding:utf8 

"""
author 
    
        yxm  created in 2016-10
"""
from geometry_msgs.msg import *
import rospy
import tf
import actionlib
import math
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from rospy.exceptions import ROSInterruptException
from tf.transformations import *
from actionlib_msgs.msg import GoalStatus 
from time import sleep
from std_msgs.msg import String,Int32,Bool
from std_srvs.srv import *
import threading
from collections import OrderedDict
from smach.state_machine import StateMachine
from smach_ros.monitor_state import MonitorState
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._PointStamped import PointStamped

target={}
dict= {
'kitchen': 0x20,
'living-room': 0x21,
'bed-room': 0x22,
'dining-room': 0x23,
'cooking-table': 0x30,
'TV-table': 0x35,
'book-cabinet': 0x40,
'dining-table': 0x45,
'sprite': 0x50,
'red-bull':0x51,
'milk': 0x52,
'tea': 0x53,
'juice': 0x54,
'coffee': 0x55,
'biscuit': 0x60,
'chips': 0x65,
'roll-paper': 0x70,
'toothpaste': 0x71
}
rospy.init_node('listener', anonymous=True)
place =''
def callback(msg):
    global place,dict
    num = hex(dict[place])
    pos = msg.pose
    px = pos.position.x
    py = pos.position.y
    pz = pos.position.z
    qx = pos.orientation.x
    qy = pos.orientation.y
    qz = pos.orientation.z
    qw = pos.orientation.w
    target[ num ] = '(Point(' + str(px) + ',' + str(py) + ',' + str(pz) + '),'  + 'Quaternion(' + str(qx) + ',' +  str(qy) + ','  + str(qz) + ',' + str(qw) + ')),' 
    return True

def listener():
    rospy.init_node('listener', anonymous=True)
    global place,target
    while (1) :
        place = raw_input('Please enter the place :')
        if place == 'end':
            break
        else:
            rospy.Subscriber("amcl_pose", PoseStamped,callback)

    target = sorted(target.iteritems(), key=lambda d:d[0])
    for tmp in target:
        print tmp 
 

if __name__ == '__main__':
    listener()
    
    
    
