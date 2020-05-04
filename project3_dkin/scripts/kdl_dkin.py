#!/usr/bin/env python

import sys
import os
import json
import rospy
import PyKDL
from tf.transformations import*
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


def callback(data):
	KDL_chain = PyKDL.Chain()
	KDL_frame = PyKDL.Frame()

	# base
	frame0 = KDL_frame.DH(A[0],Alpha[0],0,0)
	joint0 = PyKDL.Joint(PyKDL.Joint.None)
	KDL_chain.addSegment(PyKDL.Segment(joint0,frame0))

	# joint 1
	d = rospy.get_param("d1", D[0])
	frame1 = KDL_frame.DH(A[1],Alpha[1],d,Theta[0])
	joint1 = PyKDL.Joint(PyKDL.Joint.TransZ)
	KDL_chain.addSegment(PyKDL.Segment(joint1,frame1))

	# joint 2
	d = rospy.get_param("d2", D[1])
	frame2 = KDL_frame.DH(A[2],Alpha[2],d,Theta[1])
	joint2 = PyKDL.Joint(PyKDL.Joint.TransZ)
	KDL_chain.addSegment(PyKDL.Segment(joint2,frame2))

	# joint 3
	d = rospy.get_param("d3", D[2])
	frame3 = KDL_frame.DH(0,0,d,Theta[2])
	joint3 = PyKDL.Joint(PyKDL.Joint.TransZ)
	KDL_chain.addSegment(PyKDL.Segment(joint3,frame3))

	# final touches
	joint_array = PyKDL.JntArray(KDL_chain.getNrOfJoints())
	joint_array[0] = data.position[0]
	joint_array[1] = data.position[1]
	joint_array[2] = data.position[2]

	KDL_chain_solver = PyKDL.ChainFkSolverPos_recursive(KDL_chain)
	frame_end = PyKDL.Frame()
	KDL_chain_solver.JntToCart(joint_array,frame_end)
	q = frame_end.M.GetQuaternion()

	poseStamped = PoseStamped()
	poseStamped.header.frame_id = 'base_link'
	poseStamped.header.stamp = rospy.Time.now()
	poseStamped.pose.position.x = frame_end.p[0]
	poseStamped.pose.position.y = frame_end.p[1]
	poseStamped.pose.position.z = frame_end.p[2]
	poseStamped.pose.orientation.x = q[0]
	poseStamped.pose.orientation.y = q[1]
	poseStamped.pose.orientation.z = q[2]
	poseStamped.pose.orientation.w = q[3]

	marker = Marker()
	marker.header.frame_id = 'base_link'
	marker.header.stamp = rospy.Time.now()
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.pose = poseStamped.pose
	marker.scale.x = 0.06
	marker.scale.y = 0.06
	marker.scale.z = 0.06
	marker.color.a = 1
	marker.color.r = 1
	marker.color.g = 0
	marker.color.b = 0

	pubP.publish(poseStamped)
	pubM.publish(marker)

	# print('T matrices:')
	# print(T_matrices)
	# print('Final matrix:')
	# print(final_matrix)

if __name__ == '__main__':
	
	rospy.init_node('kdl_dkin', anonymous=True)

	params = {}
	pubP = rospy.Publisher('kdl_dkin/pose', PoseStamped, queue_size=10)
	pubM = rospy.Publisher('kdl_dkin/marker', Marker, queue_size=10)

	with open(os.path.dirname(os.path.realpath(__file__))[:-8] + '/yaml/dhparams.json','r') as f:
		params = json.loads(f.read())
	if (len(params) <= 0):
		print('No parameters')
		sys.exit(1)


	A, Alpha, D, Theta = {}, {}, {}, {}
	A[0], Alpha[0], D[0], Theta[0] = params['i1']
	A[1], Alpha[1], D[1], Theta[1] = params['i2']
	A[2], Alpha[2], D[2], Theta[2] = params['i3']

	f.close()

	rospy.Subscriber('joint_states', JointState, callback)
	rospy.spin()