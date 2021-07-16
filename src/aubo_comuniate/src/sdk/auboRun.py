#!/usr/bin/env python
# coding: utf-8
from robotcontrol import * 
import os.path,sys,json
import math
import numpy as np
import transforms3d as tfs

def get_matrix(x,y,z,qx,qy,qz,qw):
    rmat = tfs.quaternions.quat2mat((qw,qx,qy,qz))
    # print(tfs.quaternions.mat2quat(rmat))
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x,y,z))), rmat, [1, 1, 1])
    return rmat


def save_file(path,data):
    if str(path).startswith("~"):
        path = path.replace("~",str(os.getenv("HOME")))
    with open(path,'a') as wf:
        wf.write(str(data))
        wf.close()


def print_mat(mat):
    """
        Print mat
    """
    rot = tfs.euler.mat2euler(mat[0:3,0:3])
    return mat[0:3, 3:4].T[0],[math.degrees(rot[0]),math.degrees(rot[1]),math.degrees(rot[2])]
    # print(str(mat[0:3, 3:4].T))
    # print(str(math.degrees(rot[0]))+","+str(math.degrees(rot[1]))+","+str(math.degrees(rot[2])))


def main():
    Auboi5Robot.initialize()
    robot = Auboi5Robot()
    handle = robot.create_context()
    try:
        ip = '10.55.17.38'
        port = 8899
        result = robot.connect(ip, port)

        # 设置关节最大加速度
        robot.set_joint_maxacc((0.5,0.5, 0.5, 0.5, 0.5, 0.5))
        # 设置关节最大加速度
        robot.set_joint_maxvelc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))

        tool =  { "pos": (-0.073136, -0.007195, 0.092327), "ori": (1.0, 0.0, 0.0, 0.0) }
        tool_matrix = get_matrix(-0.063136, 0.003195, 0.092327,0.0, 0.0, 0.0, 1.0)
        inverse = np.linalg.inv(tool_matrix)
        
        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            pos = [1.02725063 ,-0.08730042  ,0.02271434]
            por = [175.00,0.0,58]
            _ori = [ math.radians(por[0]),math.radians(por[1]),math.radians(por[2]) ]
            ori = robot.rpy_to_quaternion( _ori )
            target_mat = get_matrix(pos[0],pos[1],pos[2],ori[1],ori[2],ori[3],ori[0])
            efftor_mat = np.dot(target_mat,inverse)

            a,b=print_mat(efftor_mat)
            print(a,b)
            a = [ a[0],a[1],a[2] ]
            b[0] = 175
            b[1] = 0
            a[2] += 0.05
            print (a,b)
            robot.move_to_target_in_cartesian(a,b)
            
            # now = robot.base_to_base_additional_tool(pos,ori,tool)
            # o1 = robot.quaternion_to_rpy(now['ori'])
            # print ( now['pos'] , [ math.degrees(o1[0]) ,math.degrees(o1[1]) ,math.degrees(o1[2])  ])
            
            robot.disconnect()
    except Exception as e:
       raise e
    finally:
        if robot.connected:
            robot.disconnect()
        Auboi5Robot.uninitialize()

main()