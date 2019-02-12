from ycb_renderer import YCBRenderer
import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat, mat2euler, euler2quat
from transforms3d.axangles import mat2axangle
import sys
import torch
import cv2
import os
from transforms3d.quaternions import axangle2quat, mat2quat, qmult, qinverse
from robotPose.robot_pykdl import *
def mkdir_if_missing(dst_dir):
	if not os.path.exists(dst_dir):
		os.makedirs(dst_dir)

def get_best_grasp(object_pose):
	best_pos = -1; best_poses = []
	for idx in range(pose_grasp.shape[0]):
		if energy[idx] > 45:
			continue
		pose_scene = [object_pose.dot(pose_grasp[idx])] #gripper -> object
		joint = max(min((20 + joint_val[idx])/1000, 0.04), 0) #from eigen.xml
		joint *= 180 / np.pi
		joint = np.array([joint, joint, joint]) 
		finger_poses = robot.solve_poses_from_joint(joint, 'panda_hand', pose_scene[0])
		pose_scene += finger_poses
		pose_scene = robot.offset_pose_center(pose_scene, dir='off', base_link='panda_link7') #offset hand as well
		contact_pos = (pose_scene[2] + pose_scene[3])[2, 3]/2 #two fingers
		if contact_pos > best_pos:
			best_pos = contact_pos
			best_poses = pose_scene[:]
	return best_poses

height = 480
width = 640
obj = sys.argv[1] #004_sugar_box
obj = [name for name in os.listdir('models') if name.endswith(obj)][0]
print obj
models = ['hand', 'finger', 'finger', 'camera']
base_link = 'hand'
obj_paths = [
	'panda_arm_models/{}.DAE'.format(item) for item in models]
colors = [
	[0.1*(idx+1),0,0] for idx in range(len(obj_paths))]
texture_paths = ['' for item in obj_paths]
obj_paths += ['models/{}/textured_simple.obj'.format(obj)]
texture_paths += ['models/{}/texture_map.png'.format(obj)]
colors += [[0.4,0,0]]

cls_indexes = range(len(obj_paths)) #7
renderer = YCBRenderer(width=width, height=height, render_marker=False, robot='panda_arm')
robot = robot_kinematics('panda_arm')
renderer.load_objects(obj_paths, texture_paths, colors)
renderer.set_camera_default()
renderer.set_projection_matrix(640, 480, 525, 525, 319.5, 239.5, 0.001, 1000)
pos = np.array([0, 0.3, 0])		
renderer.set_camera(pos, 2*pos, [0,0,-1])
renderer.set_light_pos(pos + np.random.uniform(-0.5, 0.5, 3))
image_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
seg_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
obj_name = '_'.join(obj.split('_')[1:])
pose_grasp = np.loadtxt('output/%s_grasp_pose.txt'%obj_name, delimiter=',')
joint_val = pose_grasp[:, -2]
energy = pose_grasp[:, -1]
pose_grasp = pose_grasp[:, :-2].reshape([pose_grasp.shape[0], 4, 4])
mkdir_if_missing('test_image')
mkdir_if_missing('test_image/%s'%obj_name)
pose_grasp[:, :3, 3] /= 1000 #scale the translation
best_grasp = np.zeros(pose_grasp.shape[0])
for idx in range(pose_grasp.shape[0]):
	object_pose = np.eye(4)
	quat = np.random.randn(4)
	object_pose[:3,:3] = quat2mat(quat / np.linalg.norm(quat))
	pose_scene = get_best_grasp(object_pose)
	pose_scene += [object_pose]
	poses = []
	for i in range(len(pose_scene)):
		tf = pose_scene[i]
		trans = tf[:3, 3]
		rot = mat2quat(tf[:3, :3])
		poses.append(np.hstack((trans,rot)))
	renderer.set_poses(poses)
	renderer.render(cls_indexes, image_tensor, seg_tensor)
	image_tensor = image_tensor.flip(0)
	seg_tensor = seg_tensor.flip(0)

	im = image_tensor.cpu().numpy()
	im = np.clip(im, 0, 1)
	im = im[:, :, (2, 1, 0)] * 255
	image = im.astype(np.uint8)

	mask = cv2.cvtColor(seg_tensor.cpu().numpy(), cv2.COLOR_RGB2BGR) #
	test_img = np.zeros(image.shape)
	test_img[mask[:,:,2]!=0] = image[mask[:,:,2]!=0]
	cv2.imwrite('test_image/%s/%06d.png'%(obj_name, idx),test_img)
	theta = 0; z = 0;
	while len(sys.argv) > 2:
		renderer.render(cls_indexes, image_tensor, seg_tensor)
		image_tensor = image_tensor.flip(0)
		seg_tensor = seg_tensor.flip(0)
		frame = [image_tensor.cpu().numpy(), seg_tensor.cpu().numpy()]
		centers = renderer.get_centers()
		for center in centers:
			x = int(center[1] * width)
			y = int(center[0] * height)
			frame[0][y-2:y+2, x-2:x+2, :] = 1
			frame[1][y-2:y+2, x-2:x+2, :] = 1
		cv2.imshow('test', cv2.cvtColor(
			np.concatenate(frame, axis=1), cv2.COLOR_RGB2BGR))
		q = cv2.waitKey(16)
		if q == ord('w'):
			z += 0.05
		elif q == ord('s'):
			z -= 0.05
		elif q == ord('a'):
			theta -= 0.1
		elif q == ord('d'):
			theta += 0.1
			#print pos + np.array([np.sin(theta), z, np.cos(theta)])
		elif q == ord('q'):
			break

		cam_pos = pos + np.array([z * np.sin(theta), z, z * np.cos(theta)])
		renderer.set_camera(cam_pos, 2 * cam_pos, [0, 0, -1])
		renderer.set_light_pos(cam_pos)