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


# Recieves movement information from the dvl and updates robot position
def dvl_callback(data):
	print "recieved"
	
	# initialize the callback
	if dvl_callback.intialized is False:
		dvl_callback.last = data.header.stamp
		dvl_callback.intialized = True
		dvl_callback.path = Path()
		dvl_callback.path.header.frame_id="odom"

		broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "body", "odom")
		return

	# Find timedifference in seconds from last incoming message
	dur = rospy.Duration()
	dur = (data.header.stamp - dvl_callback.last).to_sec()
	dvl_callback.last = data.header.stamp

	# Convert data from dvl into movement based on velocity and duration,
	# Tries to use bottom measurement first, then water measuremetns, if neither are available return.
	ps = PointStamped()
	ps.header.frame_id = "body"
	if data.bottomok == 1:
		ps.point.x = dur * data.vbx / 100.0
		ps.point.y = dur * data.vby / 100.0
	elif data.waterok == 1:
		ps.point.x = dur * data.vwx / 100.0
		ps.point.y = dur * data.vwy / 100.0
	else:
		print "no dvl measurement"
		return

	# Slow moving ROV, should not be able to move more than 1m per second
	if np.sqrt(ps.point.x**2 + ps.point.y**2) / dur > 1.0:
		print "Movement to big, returnin"
		return

	print np.sqrt(ps.point.x**2 + ps.point.y**2) / dur

	# convert movement in body coordinate into new movement in odom coordinate (world)
	# print "before", ps.point.x, ps.point.y
	ps = listener.transformPoint("odom", ps)
	# print "after", ps.point.x, ps.point.y
	# print "#################################"
	# print "#################################"

	# set new found coordinate as current coordinate
	current.position.x = ps.point.x
	current.position.y = ps.point.y
	quad = tf.transformations.quaternion_from_euler(0, 0, data.yaw)	
	current.orientation.x = quad[0]
	current.orientation.y = quad[1]
	current.orientation.z = quad[2]
	current.orientation.w = quad[3]

	# Broadcast new transformation
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "body", "odom")

	path_now = PoseStamped()
	path_now.pose.position.x = current.position.x
	path_now.pose.position.y = current.position.y
	path_now.pose.position.z = 0
	dvl_callback.path.poses.append(path_now)
	path.publish(dvl_callback.path)


# Recieves message 10 times per second to publish transformation again
# For vizualization in rviz
def tfupdate_callback(data):
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "body", "odom")


if __name__ == '__main__':
	#set up the node
	rospy.init_node('moveROV', anonymous=True)
	#make a broadcaster foir the tf frame
	broadcaster = tf.TransformBroadcaster()
	listener = tf.TransformListener()
	
	#make intilial values
	current = Pose()
	current.position.x = 1
	current.position.y = 0
	current.position.z = 0
	current.orientation.x = 0
	current.orientation.y = 0
	current.orientation.z = 0
	current.orientation.w = 1
	dvl_callback.intialized = False
	dvl_callback.last = rospy.Time()
	now = rospy.Time()
	path = rospy.Publisher('path', Path, queue_size=1)

	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "body", "odom")

	rospy.Subscriber("/abandonedMarina/DVLData", amDVL, dvl_callback);
	rospy.Subscriber("/tfupdate", Bool, tfupdate_callback);
	rospy.spin()