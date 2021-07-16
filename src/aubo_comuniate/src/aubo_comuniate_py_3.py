#!/usr/bin/env python3
# coding: utf-8
import rospy,json
from geometry_msgs.msg import Pose,PoseStamped
import os
from sdk.robotcontrol import *

Auboi5Robot.initialize()
robot = Auboi5Robot()
handle = robot.create_context()


tool =  { "pos": (-0.060354, 0.000651, 0.089300), "ori": (1.0, 0.0, 0.0, 0.0) }


def get_data():
	global Auboi5Robot
	try:
		data = ""
		r=robot.get_current_waypoint()
		r = robot.base_to_base_additional_tool(r['pos'],r['ori'],tool)
		data =  json.dumps(r) + "\n"
		return data
	except Exception as e:
		robot.disconnect()
		if robot.connected:
			robot.disconnect()
		Auboi5Robot.uninitialize()
	finally:
		pass
		

def main():
    rospy.init_node("aubo_communiate", anonymous=False)
    pub = rospy.Publisher("aubo_pose",PoseStamped,queue_size=1)
    rate = rospy.Rate(5)
    ip = rospy.get_param("/aubo_comuniate/aubo_host")
    port = 8899
    result = robot.connect(ip, port)
    if result != RobotErrorType.RobotError_SUCC:
    	logger.info("connect server{0}:{1} failed.".format(ip, port))

    while not rospy.is_shutdown():
        data = get_data()
        data = json.loads(data)
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = data['pos']
        pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z = data['ori']
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    main()
