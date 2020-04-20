#!/usr/bin/env python

import json
from tf.transformations import*

xAxis = (1, 0, 0)
yAxis = (0, 1, 0)
zAxis = (0, 0, 1)

def convert():
	with open('../yaml/dhparams.json', 'r') as f:
		params = json.loads(file.read())

	with open('../yaml/urdf.yaml','w') as f:
		for keiy in params.key():

			a, alpha, d, theta = params[key]
			a = float(a)
			alpha = float(alpha)
			d = float(d)
			theta = float(theta)

			rx = rotation_matrix(alpha, xAxis)
			tx = translation_matrix((a,0,0))
			rz = rotation_matrix(theta, zAxis)
			tz = translation_matrix((0,0,d))

			out = concatenate_matrices(rx,tx,rz,tz)
			rpy = euler_from_matrix(out)
			xyz = translation_from_matrix(out)

			file.write(key + ":\n")
			file.write("	xyz=\"{} {} {}\"".format(*xyz))
			file.write("	rpy=\"{} {} {}\"".format(*rpy))


if __name__ == '__main__':
	convert();
	print 'success';