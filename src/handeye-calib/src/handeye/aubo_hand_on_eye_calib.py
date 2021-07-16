#!/usr/bin/env python2
# coding:utf-8
import rospy
import transforms3d as tfs
from geometry_msgs.msg import PoseStamped
from handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
import math
import time,datetime
import file_operate
from tabulate import tabulate


real_aubo_pose = None
real_camera_pose = None


def aubo_callback(pose):
    global real_aubo_pose
    real_aubo_pose = pose.pose


def camera_callback(pose):
    global real_camera_pose
    real_camera_pose = pose.pose


def get_pose_from_ros(pose):
    eulor = tfs.euler.quat2euler(
        (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
    real_pose = [pose.position.x, pose.position.y, pose.position.z,
                 math.degrees(eulor[0]), math.degrees(eulor[1]), math.degrees(eulor[2])]
    return real_pose


def get_csv_from_sample(samples):
    data = ""
    for d in samples:
        data += str("hand,"+str(get_pose_from_ros(d['robot']))[1:-1]+"\n")
        data += str("eye,"+str(get_pose_from_ros(d['optical']))[1:-1]+"\n")
    return data


def calculate(samples,hand_calib):
    esti_pose = {}
    save_data = ""
    if len(samples) > 2:
        data =  [['algoritihms','x','y','z','rx','ry','rz',"distance"]]
        for algoram in hand_calib.AVAILABLE_ALGORITHMS:
            pose,final_pose = hand_calib.compute_calibration(samples,algorithm=algoram)
            data.append([algoram,pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],hand_calib._distance(pose[0],pose[1],pose[2])])
            esti_pose[algoram] = final_pose

        print "\n"+tabulate(data,headers="firstrow") + "\n"
        save_data  += str(  "\n"+tabulate(data,headers="firstrow") + "\n")

        test_result =  hand_calib._test_data(data[1:])
        data = [['name','x','y','z','rx','ry','rz',"distance"]]
        for d in test_result:
            data.append(d)
        print tabulate(data,headers="firstrow")
        save_data  += str(  "\n"+tabulate(data,headers="firstrow") + "\n")

        for algoram in hand_calib.AVAILABLE_ALGORITHMS:
            print tabulate(esti_pose[algoram],headers="firstrow")
            save_data  += str(  "\n"+tabulate(esti_pose[algoram],headers="firstrow") + "\n")
    else:
        print 'sample size not enough'
    return save_data


def save(save_data):
    result_path = "/tmp/aubo_handeye_result_"+str(datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))+".txt"
    if result_path is not None:
        file_operate.save_file(result_path,save_data)
        rospy.loginfo("Save result to  "+str(result_path))


if __name__ == '__main__':
    rospy.init_node("aubo_hand_on_eye_calib", anonymous=False)
    hand_calib = HandeyeCalibrationBackendOpenCV()
    samples = []
    aubo_pose_topic = rospy.get_param("/aubo_hand_on_eye_calib/aubo_pose_topic")
    camera_pose_topic = rospy.get_param("/aubo_hand_on_eye_calib/camera_pose_topic")
    rospy.loginfo("Get topic from param server: aubo_pose_topic:"+str(aubo_pose_topic)+" camera_pose_topic:"+str(camera_pose_topic))
   

    rospy.Subscriber(aubo_pose_topic, PoseStamped, aubo_callback,queue_size=10)
    rospy.Subscriber(camera_pose_topic, PoseStamped, camera_callback,queue_size=10)

    while not rospy.is_shutdown():
        time.sleep(1)
        if real_aubo_pose == None:
            rospy.loginfo('Waiting aubo pose topic data ...')
        elif real_camera_pose == None:
            rospy.loginfo('Waiting camera pose topic data ...')
        else:
            break


    while not rospy.is_shutdown():
        command = str(raw_input("input:  r     record,c    calculate,s     save,q    quit:"))

        if command == "r":
            samples.append( {"robot": real_aubo_pose, "optical": real_camera_pose})
            print "current sample size:"+str(len(samples))
            if len(samples) > 2:
                temp_sample,final_pose = hand_calib.compute_calibration(samples)
                print temp_sample

        elif command == 'c':
            calculate(samples,hand_calib)

        elif command == 'q':
            break

        elif command == 'p':
            print get_csv_from_sample(samples)

        elif command == 's':
            save(calculate(samples,hand_calib)+"\norigin data :\n\n"+get_csv_from_sample(samples))
        
