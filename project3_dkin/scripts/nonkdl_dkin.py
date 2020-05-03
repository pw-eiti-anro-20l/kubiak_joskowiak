#!/usr/bin/env python

import sys
import os
import json
import rospy
from tf.transformations import*
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


def callback(data):
	xAxis = (1,0,0)
	zAxis = (0,0,1)


	# joint 1
	a, alpha, d, theta = params['i1']
	a, alpha, d, theta = float(a), float(alpha), float(d), float(theta)
	d = rospy.get_param("d1", d)

	Tx = translation_matrix((a,0,0))
	Rx = rotation_matrix(alpha, xAxis)
	Tz = translation_matrix((0,0,data.position[0]+d))
	Rz = rotation_matrix(theta, zAxis)

	T_1 = concatenate_matrices(Tx,Rx,Tz,Rz)

	# joint 2
	a, alpha, d, theta = params['i2']
	a, alpha, d, theta = float(a), float(alpha), float(d), float(theta)
	d = rospy.get_param("d2", d)

	Tx = translation_matrix((a,0,0))
	Rx = rotation_matrix(alpha, xAxis)
	Tz = translation_matrix((0,0,data.position[1]+d))
	Rz = rotation_matrix(theta, zAxis)

	T_2 = concatenate_matrices(Tx,Rx,Tz,Rz)

	# joint 3
	a, alpha, d, theta = params['i3']
	a, alpha, d, theta = float(a), float(alpha), float(d), float(theta)
	d = rospy.get_param("d3", d)

	Tx = translation_matrix((a,0,0))
	Rx = rotation_matrix(alpha, xAxis)
	Tz = translation_matrix((0,0,data.position[2]+d))
	Rz = rotation_matrix(theta, zAxis)

	T_3 = concatenate_matrices(Tx,Rx,Tz,Rz)

	# final touches
	final_matrix = concatenate_matrices(T_1,T_2,T_3)
	# print(final_matrix)
	x, y, z = translation_from_matrix(final_matrix)
	qx, qy, qz, qw = quaternion_from_matrix(final_matrix)

	poseStamped = PoseStamped()
	poseStamped.header.frame_id = 'base_link'
	poseStamped.header.stamp = rospy.Time.now()
	poseStamped.pose.position.x = x
	poseStamped.pose.position.y = y
	poseStamped.pose.position.z = z
	poseStamped.pose.orientation.x = qx
	poseStamped.pose.orientation.y = qy
	poseStamped.pose.orientation.z = qz
	poseStamped.pose.orientation.w = qw

	marker = Marker()
	marker.header = poseStamped.header
	marker.type = marker.CUBE
	marker.action = marker.ADD
	marker.pose = poseStamped.pose
	marker.scale.x = 0.05
	marker.scale.y = 0.05
	marker.scale.z = 0.05
	marker.color.a = 1
	marker.color.r = 0
	marker.color.g = 1
	marker.color.b = 0

	pubP.publish(poseStamped)
	pubM.publish(marker)

	# print('T matrices:')
	# print(T_matrices)
	# print('Final matrix:')
	# print(final_matrix)

if __name__ == '__main__':
	
	rospy.init_node('nonkdl_dkin', anonymous=True)

	params = {}
	pubP = rospy.Publisher('nonkdl_dkin/pose', PoseStamped, queue_size=10)
	pubM = rospy.Publisher('nonkdl_dkin/marker', Marker, queue_size=10)

	with open(os.path.dirname(os.path.realpath(__file__))[:-8] + '/yaml/dhparams.json','r') as f:
		params = json.loads(f.read())
	if (len(params) <= 0):
		print('No parameters')
		sys.exit(1)

	rospy.Subscriber('joint_states', JointState, callback)
	rospy.spin()