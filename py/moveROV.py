#!/usr/bin/env python


import rospy
import roslib
import math
import tf
#import outlier_filter
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, PointStamped, TransformStamped, Transform
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from abandonedmarina.msg import amDVL
from numpy import mean, std
import sys
import numpy as np
import random
from tf.transformations import euler_from_quaternion, quaternion_matrix


def dvl_callback(data):
	print "recieved"
	
	if dvl_callback.intialized is False:
		dvl_callback.last = data.header.stamp
		dvl_callback.intialized = True
		return

	dur = rospy.Duration()
	dur = data.header.stamp - dvl_callback.last
	dvl_callback.last = data.header.stamp
	print dur.to_sec()
	ps = PointStamped()



if __name__ == '__main__':
	#set up the node
	rospy.init_node('moveROV', anonymous=True)
	#make a broadcaster foir the tf frame
	broadcaster = tf.TransformBroadcaster()
	listener = tf.TransformListener()
	
	#make intilial values
	current = Pose()
	current.position.x = 0
	current.position.y = 0
	current.position.z = 0
	current.orientation.x = 0
	current.orientation.y = 0
	current.orientation.z = 0
	current.orientation.w = 0
	dvl_callback.intialized = False
	dvl_callback.last = rospy.Time()
	now = rospy.Time()
	#send the tf frame
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "odom", "sonar" )

	#listen for information

	# pub = rospy.Publisher('update_buffer', Bool, queue_size=1)
	# path = rospy.Publisher('path', Path, queue_size=1)
	rospy.Subscriber("/abandonedMarina/DVLData", amDVL, dvl_callback);
	rospy.spin()