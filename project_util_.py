#!/usr/bin/env python
from transforms3d import quaternions, euler
from numpy.linalg import inv
import numpy as np
import vtk
#from flow_lib import * 
def z_buffer_to_depth(z_b, near, far):
	"""
	unit in mm, from opengl z buffer formula
	"""
	z_n = 2.0 * z_b - np.ones(z_b.shape)
	a =  np.full(z_n.shape,far + near)
	b =  np.full(z_n.shape,far * near)
	z_e = 2.0 * b / ( a - z_n * (far - near))
	z_e[z_e > 0.99*far] = 0
	z_e = (z_e*10000).astype(np.uint16)
	#print z_e.max()
	return  z_e


def ros2np(transform, t_type='mat'): #4x4
	M = np.empty((4, 4))
	if t_type=='rt':
		t = np.array(transform[0]);
		transform[1].insert(0,transform[1].pop(3))#switch order
		R = quaternions.quat2mat(transform[1])
		M[:3, :3] = R
		M[:3, 3] = t
	if t_type=='mat':

		M[:3, :4] = transform
		# M[:3, 3] = t

	M[3, :] = [0, 0, 0, 1]
	return M

def relative_transform(transform, new_transform):
	# transform_inv = se3_inverse(transform[:4,:3])
	# rel_transform = se3_mul(new_transform[:4,:3],transform_inv)
	# M = np.empty((4, 4))	
	# M[:4, :3] = rel_transform
	# M[3, :] = [0, 0, 0, 1]
	# return M #in vtk form 4x4
	return np.dot(new_transform,inv(transform))

def compute_vtk_transform(transform):
	"""
	return the vtk version of the user transform matrix
	:param [R,T ]		4x4 np array
		   [1110]
	"""
	transforms = vtk.vtkTransform()
	transforms.SetMatrix(transform.flatten().tolist())
	transforms.PostMultiply()
	return transforms

# written by Yu
def se3_inverse(RT):
	"""
	return the inverse of a RT
	:param RT=[R,T], 4x3 np array
	:return: RT_new=[R,T], 4x3 np array
	"""
	R = RT[0:3, 0:3]
	T = RT[0:3, 3].reshape((3,1))
	RT_new = np.zeros((3, 4), dtype=np.float32)
	RT_new[0:3, 0:3] = R.transpose()
	RT_new[0:3, 3] = -1 * np.dot(R.transpose(), T).reshape((3))
	return RT_new

def se3_mul(RT1, RT2):
	"""
	concat 2 RT transform
	:param RT1=[R,T], 4x3 np array
	:param RT2=[R,T], 4x3 np array
	:return: RT_new = RT1 * RT2
	"""
	R1 = RT1[0:3, 0:3]
	T1 = RT1[0:3, 3].reshape((3,1))

	R2 = RT2[0:3, 0:3]
	T2 = RT2[0:3, 3].reshape((3,1))

	RT_new = np.zeros((3, 4), dtype=np.float32)
	RT_new[0:3, 0:3] = np.dot(R1, R2)
	T_new = np.dot(R1, T2) + T1
	RT_new[0:3, 3] = T_new.reshape((3))
	return RT_new

def point_flow(K,x,y,transform,depth):
	point = back_project(K,x,y,depth)
	p_r = np.array([point[2],-point[0],-point[1]])#change to in robotic coordinates
	new_x, new_y = project(K, p_r, transform)
	return new_x - x, new_y - y # as u and v

def back_project(K,x,y,depth):
	p_camera = np.array([x,y,1])
	point = inv(K).dot(p_camera)*depth
	return point

def project_(K, point, transform): #might need to change the xyz 3d coordinates
	image = np.zeros((480,640,3), np.uint8)
	rotation = transform[:3, :3]
	translation = transform[:3, 3]
	transformed_points = rotation.dot(point)+translation#[:,np.newaxis]
	p_camera = K.dot(transformed_points)
	p_camera = (p_camera/p_camera[2]).astype(np.uint16)
	if (p_camera[0]<640) & (p_camera[1]<480) & (p_camera[0]>=0) & (p_camera[1]>=0):
		return p_camera[0], p_camera[1]
	else:
		return 0 #occulusion

def arm_part(arm_map, x, y):
	"""
	return the index of the part this pixel belongs to
	"""
	return arm_map[x,y][0] / 20 #should have better way. 360 divides into 18 regions

def compute_flow(depth, flow_list, arm_map): 
	"""
	calculate the flow image from the current projection arm map and depth.
	"""
	flow_img = np.zeros((480,640,3), np.uint8)
	for x in range(depth.shape[0]):
		for y in range(depth.shape[0]):
			transform = flow_list[arm_part[arm_map,x,y]]
			flow_img[x,y,0],flow_img[x,y,1] = point_flow[K,x,y,transform,depth[x,y]]#u and v
	#img = flow_to_image(flow_img)
	#cv2.imwrite("flow_test.png", img)