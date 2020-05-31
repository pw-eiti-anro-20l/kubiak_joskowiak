#!/usr/bin/env python

import os
import sys
import json
import rospy
import copy
from tf.transformations import*
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


def publish_joint_states(states):
	global default_states_msg
	global states_publisher
	pub_m = default_states_msg
	pub_m.header.stamp = rospy.get_rostime()
	pub_m.position = states
	print(states)

	states_publisher.publish(pub_m)


def callback(data):
	global A, Alpha, D, Theta

	z = data.pose.position.z
	y = data.pose.position.y
	x = data.pose.position.x

	d1 = rospy.get_param("d1", D[0])
	d2 = rospy.get_param("d2", D[1])
	d3 = rospy.get_param("d3", D[2])

	dz = d1-z
	dy = d2-(-y)
	dx = d3-x

	if dz > d1 or dz < 0 or dy > d2 or dy < 0 or dx > d3 or dx < 0:
		rospy.logerr('Unable to reach')
		return

	publish_joint_states([-dz,-dy,-dx,0])


def main():
	global default_states_msg
	global states_publisher
	global A, Alpha, D, Theta
	rospy.init_node('ikin', anonymous=True)

	params = {}
	states_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

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

	default_states_msg = JointState()
	default_states_msg.name = ['joint1', 'joint2', 'joint3', 'tool_joint']
	default_states_msg.position = [0.0]*4
	states_publisher.publish(default_states_msg)

	rospy.Subscriber('/oint/coordinate_system1_pose', PoseStamped, callback)
	rospy.spin()


if __name__ == '__main__':
	main()

