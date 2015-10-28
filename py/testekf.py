#!/usr/bin/env python
import rospy
import roslib

import ekf
import math
#import outlier_filter
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, PointStamped, TransformStamped, Transform
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from abandonedmarina.msg import amDVL, amMTi
from numpy import mean, std
import sys
import numpy as np
import random
import tf
from tf.transformations import euler_from_quaternion, quaternion_matrix

def dvl_callback(data):
	if dvl_callback.initialized is False:
		dvl_callback.H = np.matrix([[0,0,0,0,1,0,0,0],
									[0,0,0,0,0,1,0,0],
									[0,0,0,0,0,0,1,0],
									[0,0,1,0,0,0,0,0]])
		dvl_callback.initialized = True

	ekffilter.predict(data.header.stamp)

	ps = PointStamped()
	ps.header.frame_id = "dvl"
	if data.bottomok == 1:
		ps.point.x = data.vbx / 100.0
		ps.point.y = data.vby / 100.0
	elif data.waterok == 1:
		ps.point.x = data.vwx / 100.0
		ps.point.y = data.vwy / 100.0
	else:
		print "no dvl measurement"
		return
	ps = listener.transformPoint("body", ps)

	measurement = np.zeros((4,1))
	measurement[0] = ps.point.x
	measurement[1] = ps.point.y
 
	ekffilter.update(measurement, dvl_callback.H)
	xpos,ypos,zpos,yawpos = ekffilter.returnState()
	broadcaster.sendTransform( (xpos,ypos,zpos), tf.transformations.quaternion_from_euler(0, 0, yawpos),rospy.Time.now(), "testekf","odom")
	# print state.T

def mti_callback(data):
	if mti_callback.initialized is False:
		mti_callback.H = np.matrix([[0,0,0,1,0,0,0,0]])
		mti_callback.initialized = True
	measurement = np.matrix([data.Yaw])

	# print "first: ", ekffilter.returnState().T
	ekffilter.predict(data.header.stamp)
	
	# print "predict: ", ekffilter.returnState().T
	ekffilter.update(measurement, mti_callback.H)
	
	# print "update: ", ekffilter.returnState().T
	xpos,ypos,zpos,yawpos = ekffilter.returnState()
	
	# print "variables: ", x,y,z,theta
	# print "finished: ", ekffilter.returnState().T
	# print " "
	# print " "
	# print " "

	broadcaster.sendTransform( (xpos,ypos,zpos), tf.transformations.quaternion_from_euler(0, 0, yawpos),rospy.Time.now(), "testekf","odom")

if __name__ == '__main__':
	#set up the node
	rospy.init_node('testekf', anonymous=True)

	# tn = rospy.Time.now()

	broadcaster = tf.TransformBroadcaster()
	listener = tf.TransformListener()

	ekffilter = ekf.ekf()
	# tn = tn+rospy.Duration.from_sec(1)
	# ekffilter.predict(tn)
	# print ekffilter.returnState()
	# measurement = np.zeros((4,1))
	# measurement[0] = 1.0
	# H = np.matrix([[0,0,0,0,1,0,0,0],
	# 								[0,0,0,0,0,1,0,0],
	# 								[0,0,0,0,0,0,1,0],
	# 								[0,0,1,0,0,0,0,0]])
	# ekffilter.update(measurement, H)
	# print ekffilter.returnState()
	# tn = tn+rospy.Duration.from_sec(1)
	# fil.predict(tn)
	# print fil.returnState()
	# tn = tn+rospy.Duration.from_sec(1)
	# fil.predict(tn)
	# print fil.returnState()
	# tn = tn+rospy.Duration.from_sec(1)
	# fil.predict(tn)
	# print fil.returnState()
	# tn = tn+rospy.Duration.from_sec(1)
	# fil.predict(tn)
	# print fil.returnState()
	# 
	broadcaster.sendTransform( (0,0,0), (0,0,0,1),rospy.Time.now(), "test", "odom")

	dvl_callback.initialized = False
	mti_callback.initialized = False

	xpos = 0
	ypos = 0
	zpos = 0
	yawpos = 0

	rospy.Subscriber("/abandonedMarina/DVLData", amDVL, dvl_callback);
	rospy.Subscriber("/abandonedMarina/MTiData", amMTi, mti_callback);
	rospy.spin()
