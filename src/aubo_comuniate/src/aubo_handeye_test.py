#!/usr/bin/env python3
# coding: utf-8
from sdk.robotcontrol import *
import time
import os
import math 
import numpy as np
import vision
import calculate

def run(robot,pose):
    tool =  { "pos": (-0.060354, 0.000651, 0.089300), "ori": (1.0, 0.0, 0.0, 0.0) }
    r = robot.get_current_waypoint()
    r = robot.base_to_base_additional_tool( r['pos'], r['ori'],tool)
    r1 = calculate.getTarget(
        [
            r['pos'],
            r['ori']
        ],
        [
            pose['pos'],
            pose['ori']
        ]
    )
    pos,rpy = calculate.get_cartesian_Target( robot, r1)

    pos[2] += 0.20
    robot.move_to_target_in_cartesian(pos,rpy)

    pos[2] -= 0.20
    robot.move_to_target_in_cartesian(pos,rpy)


def test_robot():
    # 初始化logger
    logger_init()
    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))
    # 系统初始化
    Auboi5Robot.initialize()
    # 创建机械臂控制类
    robot = Auboi5Robot()
    # 创建上下文
    handle = robot.create_context()
    try:
        # 链接服务器
        ip = '10.55.130.223'
        port = 8899
        result = robot.connect(ip, port)
        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 设置碰撞等级
            #robot.set_collision_class(7)
            # 设置关节最大加速度
            robot.set_joint_maxacc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
            # 设置关节最大加速度
            robot.set_joint_maxvelc((0.1, 0.1, 0.1, 0.1, 0.1, 0.1))
            #取箱位姿信息
            move_fetch = vision.connectScanner('box')
            print("pose_num",move_fetch['num'])
            if move_fetch['num']==0 :
                print("pose_num error !")
            else:
                xyz=[move_fetch['pose_list'][0]['x'],move_fetch['pose_list'][0]['y'],move_fetch['pose_list'][0]['z']]
                print("xyz",xyz)
                rx,ry,rz= math.radians(move_fetch['pose_list'][0]['rx']), math.radians(move_fetch['pose_list'][0]['ry']), math.radians(move_fetch['pose_list'][0]['rz'])
                quater=robot.rpy_to_quaternion([rx,ry,rz])
                pose={"pos":xyz,"ori":quater}
                print("pose",pose)
                run(robot,pose)
            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))
    finally:
        if robot.connected:
            logger.info("{0} test shutdown and disconnect.".format(Auboi5Robot.get_local_time()))
            robot.disconnect()
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

test_robot()
