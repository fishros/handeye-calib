#!/usr/bin/env python
# coding: utf-8

import socket
import threading
import time
import json
import rospy
import Queue
import transforms3d as tfs
import math
from geometry_msgs.msg import PoseStamped

def get_pose_from_ros(pose):
    eulor = tfs.euler.quat2euler((pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z))
    real_pose = [pose.position.x,pose.position.y,pose.position.z,eulor[0]/math.pi*180,eulor[1]/math.pi*180,eulor[2]/math.pi*180]
    return real_pose


def tcplink(sock, addr,recv_pose_queue):
    rospy.loginfo('Accept new connection from %s:%s...' % addr)
    while True:
        data = sock.recv(1024)
        rospy.loginfo('recvdata:[%s]' % data.decode('utf-8'))
        time.sleep(0.1)
        if not data or data.decode('utf-8') == 'exit':
            break
        data2=data.decode('utf-8')
        data2=json.loads(data2)

        if data2['obj_type']=="box":
            if recv_pose_queue.qsize() == 0:
                sock.send(json.dumps({"msg_type":"location_rsp","obj_type":"box","num":0,"pose_list":[]}).encode("utf-8"))
            else:
                pose = recv_pose_queue.get()
                if time.time()-pose.header.stamp.secs>3:
                    sock.send(json.dumps({"msg_type":"location_rsp","obj_type":"box","num":0,"pose_list":[]}).encode("utf-8"))
                pose = get_pose_from_ros(pose.pose)
                respData0={"msg_type":"location_rsp","obj_type":"box","num":1,"pose_list":[{"x":pose[0],"y":pose[1],"z":pose[2],"rx":pose[3],"ry":pose[4],"rz":pose[5]}]}
                respData1=json.dumps(respData0)
                sock.send(respData1.encode("utf-8"))
        elif data2['obj_type']=="bracket":
            pass
        else:    
            sock.send(('recv data:[%s]' % data.decode('utf-8')).encode('utf-8'))

    sock.close()
    rospy.loginfo('Connection from %s:%s closed.' % addr)


recv_pose_queue = Queue.Queue(1)

def callback(pose):
    global recv_pose_queue
    if recv_pose_queue.full():
        recv_pose_queue.get()
    recv_pose_queue.put(pose)


if __name__=='__main__':
    rospy.init_node("aruco_tcp_publisher", anonymous=False)
    rospy.Subscriber("/aruco_single/pose", PoseStamped, callback)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 8800))
    server.listen(5)
    while not rospy.is_shutdown():
        sock, addr = server.accept()
        t = threading.Thread(target=tcplink, args=(sock, addr,recv_pose_queue))
        t.start()