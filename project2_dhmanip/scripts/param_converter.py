#!/usr/bin/env python

import json
from tf.transformations import*

xAxis = (1, 0, 0)
yAxis = (0, 1, 0)
zAxis = (0, 0, 1)

def convert():
	with open('/home/sirlurksal0t/catkin_ws/src/kubiak_joskowiak/project2_dhmanip/yaml/dhparams.json', 'r') as f:
		params = json.loads(f.read())

	with open('/home/sirlurksal0t/catkin_ws/src/kubiak_joskowiak/project2_dhmanip/yaml/urdf.yaml','w') as f:
		for key in params.keys():

			a, alpha, d, theta = params[key]
			a = float(a)
			alpha = float(alpha)
			d = float(d)
			theta = float(theta)

			rx = rotation_matrix(alpha, xAxis)
			tx = translation_matrix((a,0,0))
			rz = rotation_matrix(theta, zAxis)
			tz = translation_matrix((0,0,d))

			out = concatenate_matrices(tz,rz,tx,rx)
			rpy = euler_from_matrix(out)
			xyz = translation_from_matrix(out)

			f.write(key + ":\n")
			f.write("	xyz=\"{} {} {}\"\n".format(*xyz))
			f.write("	rpy=\"{} {} {}\"\n".format(*rpy))


if __name__ == '__main__':
	convert();
	print 'success';