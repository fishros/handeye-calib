#!/usr/bin/env python
# coding: utf-8
import rospy
import transforms3d as tfs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
import math
import numpy as np

real_aubo_pose = None
real_camera_pose = None
X,Y,Z = [ 0.0433182 ,  0.01781659 ,-0.019428  ]  
RX,RY,RZ = (0.0006357429782050881, -0.0009131889099997172, 1.5589477353939896)
rmat = tfs.affines.compose(np.asarray((X,Y,Z)), tfs.euler.euler2mat(RX,RY,RZ), [1, 1, 1])  

def aubo_callback(pose):
    global real_aubo_pose
    real_aubo_pose = opencv_pose2mat(pose)


def camera_callback(pose):
    global real_camera_pose
    real_camera_pose = opencv_pose2mat(pose.pose)


def print_mat(mat):
    """
        Print mat
    """
    rot = tfs.euler.mat2euler(mat[0:3,0:3])
    rospy.loginfo(str(mat[0:3, 3:4].T))
    rospy.loginfo(str(math.degrees(rot[0]))+","+str(math.degrees(rot[1]))+","+str(math.degrees(rot[2])))


def opencv_pose2mat(pose): 
    """
        opencv Pose msg to rot.
    """
    rot = tfs.quaternions.quat2mat((pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z))
    rmat = tfs.affines.compose(np.asarray((pose.position.x,pose.position.y,pose.position.z)), rot, [1, 1, 1])  
    return rmat

if __name__ == '__main__':
    rospy.init_node("test_aubo", anonymous=False)
    aubo_pose_topic = rospy.get_param("/test_aubo/aubo_pose_topic")
    camera_pose_topic = rospy.get_param("/test_aubo/camera_pose_topic")

    rospy.loginfo("Get topic from param server: aubo_pose_topic:"+str(aubo_pose_topic)+" camera_pose_topic:"+str(camera_pose_topic))
    rospy.Subscriber(aubo_pose_topic, Pose, aubo_callback)
    rospy.Subscriber(camera_pose_topic, PoseStamped, camera_callback)

    while not rospy.is_shutdown():
        command = str(raw_input("c to calculate,q to quit:"))
        if command == "c" :
            real_pose = np.dot(real_aubo_pose,rmat)
            real_pose = np.dot(real_pose,real_camera_pose)
            print_mat(real_pose)
        elif command =="p":
            print real_aubo_pose,real_camera_pose
        elif command=='q' :
            break;

