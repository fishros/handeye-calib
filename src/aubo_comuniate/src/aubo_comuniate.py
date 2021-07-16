#!/usr/bin/env python
# coding: utf-8
import subprocess,sys,signal
import rospy,json
from geometry_msgs.msg import Pose
import os

def get_subprocess():
    p = subprocess.Popen("python3  "+os.path.dirname(__file__)+"/sdk/auboi10.py"  ,shell=True,stdout=subprocess.PIPE,stdin=subprocess.PIPE)
    get_data(p)
    return p

def get_data(p):
    try:
        returncode = p.poll()
        if returncode is None:
            p.stdin.write("r\n")
            p.stdin.flush()
            return p.stdout.readline()
    except Exception as e:
        pass


def exit_subprocess(p):
    p.send_signal()


def main():
    rospy.init_node("aubo_communiate", anonymous=False)
    pub = rospy.Publisher("aubo_pose",Pose,queue_size=1)
    rate = rospy.Rate(5)
    p = get_subprocess()
    while not rospy.is_shutdown():
        data = get_data(p)
        data = json.loads(data)
        print(data)
        pose = Pose()
        pose.position.x,pose.position.y,pose.position.z = data['pos']
        pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z = data['ori']
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    main()
    # pass
