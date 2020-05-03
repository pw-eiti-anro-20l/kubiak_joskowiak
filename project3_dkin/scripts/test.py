#!/usr/bin/env python

import json
from tf.transformations import*


xAxis = (1, 0, 0)
yAxis = (0, 1, 0)
zAxis = (0, 0, 1)

# T_matrices = []
# T_matrices.append(1)
# T_matrices.append(2)
# print(T_matrices)

# with open('/home/sirlurksal0t/catkin_ws/src/kubiak_joskowiak/project2_dhmanip/yaml/dhparams.json','r') as f:
# 	params = json.loads(f.read())
# print(len(params))

# rx = rotation_matrix(3.14/2, xAxis)
# print(rx)
# tx = translation_matrix((7,0,0))
# print(tx)
# rz = rotation_matrix(3.14/2, zAxis)
# print(rz)
# cx = concatenate_matrices(rx,tx,rz)
# print(cx)