#!/usr/bin/env python

import rospy
from time import sleep
from math import pi, sin, cos
from project5_inkin.srv import ocmd_control
from project4_intpol.srv import oint_control, oint_clear


def ocmd_handle(request):
	if request.type == 'rectangle':
		x = request.x
		y = request.y
		z = request.z
		a = request.a
		b = request.b
		t = request.t
		A = [x-a/2,y-b/2,z/2]
		B = [x+a/2,y-b/2,z/2]
		C = [x+a/2,y+b/2,z/2]
		D = [x-a/2,y+b/2,z/2]
		# E = [x-a/2,y+b/2,z+c/2]
		# F = [x-a/2,y-b/2,z+c/2]
		try:
			rospy.wait_for_service('oint_control_srv')
			oint_proxy = rospy.ServiceProxy('oint_control_srv',oint_control)
			oint_proxy('quartic',A[0],A[1],A[2],0,0,0,0)
			n = 0
			while n < request.N:
				oint_proxy('quartic',B[0],B[1],B[2],0,0,0,t)
				oint_proxy('quartic',C[0],C[1],C[2],0,0,0,t)
				oint_proxy('quartic',D[0],D[1],D[2],0,0,0,t)
				# oint_proxy('quartic',E[0],E[1],E[2],0,0,0,t)
				# oint_proxy('quartic',F[0],F[1],F[2],0,0,0,t)
				oint_proxy('quartic',A[0],A[1],A[2],0,0,0,t)
				n+=1
				print('Figure {} complete'.format(n))
			return 'All figures completed'
		except rospy.ServiceException as e:
			print('Service call failed:\n %s'%e)

	elif request.type == 'ellipse':
		resolution = request.res
		x = request.x
		y = request.y
		z = request.z
		C = [x,y,z]
		U = [1,1,1]
		V = [1,-1,-1]
		a = request.a
		b = request.b
		t = request.t
		try:
			rospy.wait_for_service('oint_control_srv')
			oint_proxy = rospy.ServiceProxy('oint_control_srv',oint_control)
			oint_proxy('quartic',C[0]+a*U[0],C[1]+a*U[1],C[2]+a*U[2],0,0,0,0)
			phi = 0.0
			while phi < 2*pi*request.N:
				A = [0,0,0]
				for i in 0,1,2:
					A[i] = C[i]+a*cos(phi)*U[i]+b*sin(phi)*V[i]
				oint_proxy('linear',A[0],A[1],A[2],0,0,0,t)
				phi+=2*pi/resolution
			return 'All figures completed'
		except rospy.ServiceException as e:
			print('Service call failed:\n %s'%e)

	else:
		return 'Usage: \'rectangle\'/\'elipse\' x y z a b N res T'

def main():
	rospy.init_node('ocmd')
	s = rospy.Service('ocmd_control_srv', ocmd_control, ocmd_handle)
	print('ocmd ready')
	rospy.spin()


if __name__ == '__main__':
	main()