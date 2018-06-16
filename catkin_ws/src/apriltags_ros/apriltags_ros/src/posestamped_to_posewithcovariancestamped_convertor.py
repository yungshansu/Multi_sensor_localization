#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

pub = rospy.Publisher('/svo/pose', PoseWithCovarianceStamped, queue_size=5)
def cb_pose(msg):
	mypose = PoseWithCovarianceStamped() 
	mypose.header = msg.header
	mypose.header.frame_id = "cam0"
	mypose.pose.pose = msg.pose
	pub.publish(mypose)



if __name__ == '__main__':
	rospy.init_node('pose_convertor', anonymous = False)
	rospy.Subscriber('/svo/pose_cam/0', PoseStamped, cb_pose, queue_size = 5)
	rospy.spin()