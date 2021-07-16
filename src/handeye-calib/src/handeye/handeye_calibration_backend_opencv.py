#!/usr/bin/env python
# coding: utf-8
import sys
import numpy as np
import transforms3d as tfs
from rospy import logerr, logwarn, loginfo
import math
import sys,os

if os.path.exists('/opt/ros/kinetic/lib/python2.7/dist-packages'):
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    import cv2
    sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
elif  os.path.exists('/opt/ros/melodic/lib/python2.7/dist-packages'):
    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
    import cv2
    sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
else:
    import cv2



class HandeyeCalibrationBackendOpenCV(object):
    MIN_SAMPLES = 2  # TODO: correct? this is what is stated in the paper, but sounds strange
    """Minimum samples required for a successful calibration."""

    AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
       # 'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    @staticmethod
    def _msg_to_opencv(transform_msg):
        if isinstance(transform_msg, dict):
            cmt = transform_msg["position"]
            tr = np.array((cmt['x'], cmt['y'], cmt['z']))
            cmq = transform_msg["orientation"]
            rot = tfs.quaternions.quat2mat(
                (cmq['w'], cmq['x'], cmq['y'], cmq['z']))
            return rot, tr
        else:
            cmt = transform_msg.position
            tr = np.array((cmt.x, cmt.y, cmt.z))
            cmq = transform_msg.orientation
            rot = tfs.quaternions.quat2mat((cmq.w, cmq.x, cmq.y, cmq.z))
            return rot, tr


    @staticmethod
    def _test_data(data):
        mean_result = ['mean']
        var_result = ['var']
        std_result = ['std']
        row_len = len(data)
        column_len = len(data[0])
        for i in range(column_len):
            temp = []
            for j in range(row_len):
                temp.append(data[j][i])
            if  isinstance(temp[0],float):
                mean_result.append(np.mean(temp))
                var_result.append(np.var(temp))
                std_result.append(np.std(temp))
        return mean_result,var_result,std_result

    @staticmethod
    def _get_opencv_samples(samples):
        """
        Returns the sample list as a rotation matrix and a translation vector.
        :rtype: (np.array, np.array)
        """
        hand_base_rot = []
        hand_base_tr = []
        marker_camera_rot = []
        marker_camera_tr = []

        for s in samples:
            camera_marker_msg = s['optical']
            (mcr, mct) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(
                camera_marker_msg)
            marker_camera_rot.append(mcr)
            marker_camera_tr.append(mct)

            base_hand_msg = s['robot']
            (hbr, hbt) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(base_hand_msg)
            hand_base_rot.append(hbr)
            hand_base_tr.append(hbt)

        return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)

    @staticmethod
    def _distance(x,y,z):
        return math.sqrt(x*x+y*y+z*z)


    def compute_calibration(self, samples, algorithm=None):
        """
        Computes the calibration through the OpenCV library and returns it.

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        if algorithm is None:
            algorithm = 'Tsai-Lenz'

        if len(samples) < HandeyeCalibrationBackendOpenCV.MIN_SAMPLES:
            logwarn("{} more samples needed! Not computing the calibration".format(
                HandeyeCalibrationBackendOpenCV.MIN_SAMPLES - len(samples)))
            return

        # Update data
        opencv_samples = HandeyeCalibrationBackendOpenCV._get_opencv_samples(samples)
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = opencv_samples

        if len(hand_world_rot) != len(marker_camera_rot):
            logerr("Different numbers of hand-world and camera-marker samples!")
            raise AssertionError

        method = HandeyeCalibrationBackendOpenCV.AVAILABLE_ALGORITHMS[algorithm]
        hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(hand_world_rot, hand_world_tr, marker_camera_rot,
                                                               marker_camera_tr, method=method)
        result = tfs.affines.compose(np.squeeze(hand_camera_tr), hand_camera_rot, [1, 1, 1])

        (rx, ry, rz) = [math.degrees(float(i)) for i in tfs.euler.mat2euler(hand_camera_rot)]
        (hctx, hcty, hctz) = [float(i) for i in hand_camera_tr]
        
        final_pose = [[algorithm,'x','y','z','rx','ry','rz',"distance"]]
        for i in range(len(samples)):
            pose1 = tfs.affines.compose(np.squeeze(hand_world_tr[i]), hand_world_rot[i], [1, 1, 1])
            pose2 = tfs.affines.compose(np.squeeze(marker_camera_tr[i]), marker_camera_rot[i], [1, 1, 1])
            temp = np.dot(pose1,result)
            temp = np.dot(temp,pose2)
            tr = temp[0:3,3:4].T[0]
            rot =tfs.euler.mat2euler(temp[0:3,0:3]) 
            final_pose.append(["point"+str(i),tr[0],tr[1],tr[2],rot[0],rot[1],rot[2],HandeyeCalibrationBackendOpenCV._distance(tr[0],tr[1],tr[2])])

        test_result = HandeyeCalibrationBackendOpenCV._test_data(final_pose[1:])
        final_pose.append(test_result[0])
        final_pose.append(test_result[1])
        final_pose.append(test_result[2])

        result_tuple = (hctx, hcty, hctz,rx,ry,rz),final_pose
        return result_tuple
