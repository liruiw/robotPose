from ycb_renderer import YCBRenderer
import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat, mat2euler, euler2quat
from transforms3d.axangles import mat2axangle
import sys
import torch
import cv2
import os
import rospy

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseArray, Pose
from transforms3d.quaternions import axangle2quat, mat2quat, qmult, qinverse
from robotPose.robot_pykdl import *

def deg2rad(deg):
    if type(deg) is list:
        return [x/180.0*np.pi for x in deg]
    return deg/180.0*np.pi

def rad2deg(rad):
    if type(rad) is list:
        return [x/np.pi*180 for x in rad]
    return rad/np.pi*180

def ros_quat(tf_quat): #wxyz -> xyzw
	quat = np.zeros(4)
	quat[-1] = tf_quat[0]
	quat[:-1] = tf_quat[1:]
	return quat

def tf_quat(ros_quat): #xyzw -> wxyz
	quat = np.zeros(4)
	quat[0] = ros_quat[-1]
	quat[1:] = ros_quat[:-1]
	return quat

def mkdir_if_missing(dst_dir):
	if not os.path.exists(dst_dir):
		os.makedirs(dst_dir)

def get_grasp_scene(pose, object_pose, joint_val):
	pose_scene = [object_pose.dot(pose)] #gripper -> object
	joint = max(min((20 + joint_val)/1000, 0.04), 0) 
	joint *= 180 / np.pi
	joint = np.array([joint, joint, joint]) 
	finger_poses = robot.solve_poses_from_joint(joint, 'panda_hand', pose_scene[0])
	pose_scene += finger_poses
	pose_scene = robot.offset_pose_center(pose_scene, dir='off', base_link='panda_link7') #offset hand as well
	return pose_scene


def example_call_back(pose_grasps):
	global current_pose_grasp
	current_pose_grasp = []
	for pose_i in pose_grasps.poses[-10:]: #avoid caching
		current_pose_grasp.append(np.array([pose_i.position.x, pose_i.position.y, pose_i.position.z , \
			pose_i.orientation.x, pose_i.orientation.y, pose_i.orientation.z, pose_i.orientation.w]))

def render_pose(curr_base_pose, poses):
	global num, mat_file
	finger_joint = 2 
	image_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
	seg_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
	obj_name = '_'.join(classes[target].split('_')[1:])
	grasp_pose = []
	joints = []
	
	mkdir_if_missing('test_image/%s' % obj_name)
	for pose_grasp in poses:
		quat = tf_quat(pose_grasp[3:])
		grasp_candidate = np.eye(4)
		grasp_candidate[:3, :3] = quat2mat(quat)
		grasp_candidate[:3, 3] = pose_grasp[:3]
		
		hand_pose = object_pose[target_idx].dot(grasp_candidate)
		grasp_candidate_r = np.linalg.inv(curr_base_pose).dot(hand_pose)

		pos = grasp_candidate_r[:3, 3]
		rot = ros_quat(mat2quat(grasp_candidate_r[:3, :3]))
		joints =  robot.inverse_kinematics(pos, rot)
		grasp_pose = get_grasp_scene(grasp_candidate, object_pose[target_idx], finger_joint)
		if joints is not None:
			break
		else:
			print 'filter'
	if joints is not None:
		p = robot.forward_position_kinematics(joint_values=np.array(joints))
		joints = rad2deg(joints)
		arm_pose = robot.solve_poses_from_joint(joints, base_link='panda_link0', base_pose=curr_base_pose)
		arm_pose = robot.offset_pose_center(arm_pose, dir='off', base_link='panda_link0')
		
		pose_scene = arm_pose + grasp_pose + object_pose # construct all poses
		poses = []
		for i in range(len(pose_scene)):
			tf = pose_scene[i]
			trans = tf[:3, 3]
			rot = mat2quat(tf[:3, :3])
			poses.append(np.hstack((trans,rot)))
		
		render_classes = range(len(poses))
		renderer.set_poses(poses)
		renderer.render(render_classes, image_tensor, seg_tensor)
		image_tensor = image_tensor.flip(0)
		seg_tensor = seg_tensor.flip(0)
		im = image_tensor.cpu().numpy()
		im = np.clip(im, 0, 1)
		im = im[:, :, (2, 1, 0)] * 255
		image = im.astype(np.uint8)

		mask = cv2.cvtColor(seg_tensor.cpu().numpy(), cv2.COLOR_RGB2BGR) #
		test_img = np.zeros(image.shape)
		test_img[mask[:,:,2]!=0] = image[mask[:,:,2]!=0]
		cv2.imwrite('test_image/%s/ros_%06d.png'%(obj_name, num),test_img)
		num += 1
	
# ros
def example_publish(curr_base_pose):
	global mat_poses, num, current_pose_grasp, object_pose
	while not rospy.is_shutdown():
		if current_pose_grasp is not None:
			render_pose(curr_base_pose, current_pose_grasp)	
			current_pose_grasp = None
		else:
			object_pose = []
			mat_file = 'YCB_Video/data/{}/{:06d}-meta.mat'.format(video, num)
			mat = sio.loadmat(mat_file)
			mat_poses = mat['poses']
			cls_indexes = mat['cls_indexes'].squeeze()
			for i, cls in enumerate(cls_indexes):
				pose = np.eye(4)
				pose[:3, :4] = mat_poses[:, :, i]
				object_pose.append(pose.copy())
			class_msg = Int32MultiArray()
			class_msg.data = cls_indexes
			classes_pub.publish(class_msg)
			target_pub.publish(target)
			msgs = PoseArray()
			for i in range(cls_indexes.shape[0]):
				msg = Pose()
				quat = mat2quat(object_pose[i][:3, :3])
				msg.orientation.x = quat[1]
				msg.orientation.y = quat[2]
				msg.orientation.z = quat[3]
				msg.orientation.w = quat[0]
				msg.position.x = object_pose[i][0, 3]
				msg.position.y = object_pose[i][1, 3]
				msg.position.z = object_pose[i][2, 3]
				msgs.poses.append(msg)
			pose_pub.publish(msgs) #publish posearray
			rate.sleep()

if __name__ == '__main__':
	height = 480
	width = 640
	video = sys.argv[1]
	num = 1
	current_pose_grasp = None
	pose_flag = False
	target = int(sys.argv[2]) 
	classes = ('__background__', '002_master_chef_can', '003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', \
							 '007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana', '019_pitcher_base', \
							 '021_bleach_cleanser', '024_bowl', '025_mug', '035_power_drill', '036_wood_block', '037_scissors', '040_large_marker', \
							 '051_large_clamp', '052_extra_large_clamp', '061_foam_brick')
	mat_file = 'YCB_Video/data/{}/000001-meta.mat'.format(video)
	mat = sio.loadmat(mat_file)
	cls_indexes = mat['cls_indexes'].squeeze()
	mat_poses = mat['poses']
	objs = []
	object_pose = []
	target_idx = -1 
	for i, cls in enumerate(cls_indexes):
		pose = np.eye(4)
		pose[:3, :4] = mat_poses[:, :, i]
		object_pose.append(pose)
		if cls == target:
			target_idx = i
		objs.append([name for name in os.listdir('models') if name.endswith(classes[cls])][0])
	models = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'hand', 'finger', 'finger', 'camera']

	obj_paths = [
		'panda_arm_models/{}.DAE'.format(item) for item in models]
	colors = [
		[0.1*(idx+1),0,0] for idx in range(len(obj_paths))]
	texture_paths = ['' for item in obj_paths]
	obj_paths += ['models/{}/textured_simple.obj'.format(obj) for obj in objs]
	texture_paths += ['models/{}/texture_map.png'.format(obj) for obj in objs]
	colors += [[0.3 + 0.1 * i,0,0] for i in range(len(objs))]
	rospy.init_node('example_publisher')
	rate = rospy.Rate(30) 
	pose_pub = rospy.Publisher('object_poses', PoseArray, queue_size=1) #add prefix
	target_pub = rospy.Publisher('grasp_target', Int32, queue_size=1)
	classes_pub = rospy.Publisher('grasp_classes', Int32MultiArray, queue_size=1)
	renderer = YCBRenderer(width=width, height=height, render_marker=False, robot='panda_arm')
	renderer.load_objects(obj_paths, texture_paths, colors)
	renderer.set_camera_default()
	renderer.set_projection_matrix(640, 480, 525, 525, 319.5, 239.5, 0.001, 1000)
	renderer.set_camera_default()
	renderer.set_light_pos(np.random.uniform(-0.5, 0.5, 3)) #random position for robot base
	robot = robot_kinematics('panda_arm')
	rospy.Subscriber('grasp_pose', PoseArray, example_call_back)
	curr_base_pose =  np.eye(4)
	curr_base_pose[:3, :] =  mat_poses[:, :, target]
	curr_base_pose[:3, 3] += np.array([-0.8, 0.2, 0.5])
	try:
		example_publish(curr_base_pose)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass