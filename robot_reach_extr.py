import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import PyKDL
import scipy.io as sio
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat, mat2euler, euler2quat
from transforms3d.axangles import mat2axangle
import os
import numpy.random as random
from numpy.linalg import inv
from ycb_renderer import YCBRenderer
import torch
from robot_pykdl import *

fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')
robot = robot_kinematics('panda_arm')
itr = 7000
x3d = np.ones((3, itr), dtype=np.float32)
color_x3d = np.ones((itr, 3), dtype=np.float32)
reach_3d = np.ones((3, itr), dtype=np.float32)
initial_pt = np.array([0,0,1])
base_link = 'panda_link0'
for i in range(itr):
    r = np.zeros(3)  
    theta = np.random.uniform(low=0, high=np.pi/2)
    phi = np.random.uniform(low=0, high=2*np.pi)
    r = np.random.uniform(low=2.3, high=2.5)
    x3d[:,i] = np.array([r*np.sin(theta)*np.cos(phi), r*np.sin(theta)*np.sin(phi), r*np.cos(theta)])
    color_x3d[i, :] = np.ones(3)
    end_effector_reach = robot.gen_rand_pose(base_link)[0][-1][:3,3]
    reach_3d[:,i] = end_effector_reach
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
max_val = 2
ax.set_xlim3d(-max_val, max_val)
ax.set_ylim3d(-max_val, max_val)
ax.set_zlim3d(-max_val, max_val)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
print x3d[0].shape,x3d[1].shape,x3d[2].shape, color_x3d.shape
ax.scatter(x3d[0],x3d[1],x3d[2], c=[1,0,0])
ax.scatter(reach_3d[0],reach_3d[1],reach_3d[2], c=[0,1,0])
plt.show()