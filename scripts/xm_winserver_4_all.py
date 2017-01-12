#!/usr/bin/env python
#encoding:utf8

"""
author :Luke Liao, robin1001
re-edit by X_Y 耀
2015-9-19
re-edit by yxm
2016-4-1

"""

import rospy
import socket
import struct
import threading
import sys
from std_msgs.msg import Int32,String
from xm_msgs.srv import *
from xm_msgs.msg import *
import cmd

class XM_winserver:
#    WIN_IP = '10.12.1.32'
    WIN_IP = '192.168.1.100'
#    WIN_IP = '192.168.0.1'

    SELF_PORT = 10000
    SPEECH_PORT = 10000
    FACE_PORT = 10002
        
    def __init__(self):
        self.speech_pub = rospy.Publisher('task_coming', xm_Task,queue_size=100)
        #self.kinect_srv = rospy.Service('kinect_srv', xm_KinectSrv, self.kinect_srv_handler)
        self.face_srv = rospy.Service('face_srv', xm_FaceSrv, self.face_srv_handler)
#         self.speech_sub = rospy.Subscriber('tts_data', String, self.speech_callback)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(('', XM_winserver.SELF_PORT))

    def start(self):
        print 'xm_winserver start...'
        print 'tts server info %s:%d' % (XM_winserver.WIN_IP, XM_winserver.SPEECH_PORT)
        print 'face server info %s:%d' % (XM_winserver.WIN_IP, XM_winserver.FACE_PORT)
#         print 'kinect server info %s:%d' % (XM_winserver.WIN_IP, XM_winserver.KINECT_PORT)
        #start win server
        t=threading.Thread(target = self.start_winserver)
        t.setDaemon(True)
        t.start()
#         while not rospy.is_shutdown():
        rospy.Subscriber('tts_data', String, self.speech_callback)
#             rospy.sleep(10)
#         rospy.spin()
            
    def start_winserver(self):
        self.server.listen(10)
        while not rospy.is_shutdown():
            sock, addr = self.server.accept()
            print "connect coming"
            t = threading.Thread(target=self.handle_connect, args=(sock, addr))
            t.setDaemon(True)
            t.start()

    def handle_connect(self, sock, addr):
        length, status = struct.unpack('!ib', self.recv_len(sock, 5))
        task=xm_Task()
        room,cmd=struct.unpack('!bb',self.recv_len(sock, 2))
#        print room,cmd
        task.status=status
        task.place=room
        task.task=cmd
        self.speech_pub.publish(task)
        print(task)

    def recv_len(self, sock, length):
        data = []
        left = length
        while left != 0:
            tmp = sock.recv(left)
            data.append(tmp)
            left -= len(tmp)
        return ''.join(data)

    def _send(self, ip, port, data):
        self._send_helper(ip, port, data).close()

    #send and wait return data
   
    def _send_wait(self, ip, port, data):
        sock = self._send_helper(ip, port, data)
        length, = struct.unpack('!i', self.recv_len(sock, 4))
        data = self.recv_len(sock, length)
        print len(data)
        sock.close()
        return data

    def _send_helper(self, ip, port, data):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((ip, port))
            data_len = struct.pack('!i', len(data))
            sock.sendall(data_len + data)
        except socket.error, ex:
            print socket.error, ex
        return sock

    def speech_callback(self, msg):
        if msg.data is not None:
            print 'new speech cmd arrived: ', msg.data,len(msg.data) 
            speech = struct.pack('!b100s',0x01,msg.data)
            self._send(XM_winserver.WIN_IP,XM_winserver.SPEECH_PORT, speech)
                
    def face_srv_handler(self, request):
        print 'new face request arrived'
        rep = xm_FaceSrvResponse()
        print request
        send_data = struct.pack('!bbb', 0x02, request.people,request.cmd)
        recv_data = self._send_wait(XM_winserver.WIN_IP, XM_winserver.FACE_PORT, send_data)
        rep.state,rep.people,rep.is_success=struct.unpack('!BBB',recv_data)
        print "return"
        print rep
        return rep
        
if __name__ == "__main__":
    rospy.init_node("xm_winserver")
    XM_winserver().start()
    rospy.spin()
