#!/usr/bin/env python
# coding: utf-8
import sys
import numpy as np
import transforms3d as tfs
import math
import file_operate
import rospy
import json
from tabulate import tabulate
from handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV


def eular_to_msg(x, y, z, rx, ry, rz):
    """
    transform cvs msg to ros type Pose 
    """
    rot = tfs.euler.euler2quat(math.radians(
        rx), math.radians(ry), math.radians(rz))
    return {"position":{"x":x,"y":y,"z":z},"orientation":{"w":rot[0],"x":rot[1],"y":rot[2],"z":rot[3]}} 

def get_samples(cal, tool):
    """
    Generate robot msgs.
    """
    samples = []
    for i in range(0, tool.shape[0], 6):
       optical = eular_to_msg(cal[i], cal[i+1], cal[i+2], cal[i+3], cal[i+4], cal[i+5])
       hand = eular_to_msg( tool[i], tool[i+1], tool[i+2], tool[i+3], tool[i+4], tool[i+5])
       samples.append({"robot": hand, "optical": optical})
    return samples


if __name__ == '__main__':
    rospy.init_node("base_hand_on_eye_calib", anonymous=False)
    file_path = rospy.get_param("/base_hand_on_eye_calib/base_handeye_data")
    result_path = rospy.get_param("/base_hand_on_eye_calib/base_handeye_result")

    hand_calib = HandeyeCalibrationBackendOpenCV()

    if file_path is not None:
        rospy.loginfo("Get param base_handeye_data file path: "+str(file_path))
        if not str(file_path).endswith(".csv"):
            rospy.logerr("The data file not a csv file :"+str(file_path))
            exit()
        
    tool, cal = file_operate.read_handeye_data(file_path)
    samples = get_samples(cal=cal, tool=tool)
    esti_pose = {}
    save_data = ""
    if len(samples) > 2:
        data =  [['algoritihms','x','y','z','rx','ry','rz',"distance"]]
        for algoram in hand_calib.AVAILABLE_ALGORITHMS:
            pose,final_pose = hand_calib.compute_calibration(samples,algorithm=algoram)
            data.append([algoram,pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],hand_calib._distance(pose[0],pose[1],pose[2])])
            esti_pose[algoram] = final_pose
        print  str("\n"+tabulate(data,headers="firstrow") + "\n")
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
            
        if result_path is not None:
            file_operate.save_file(result_path,save_data)
            rospy.loginfo("Save result to  "+str(result_path))