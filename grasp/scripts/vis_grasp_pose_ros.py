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

def call_back(pose_grasp):
	models = ['hand', 'finger', 'finger', 'camera']
	base_link = 'hand'
	obj_paths = [
		'panda_arm_models/{}.DAE'.format(item) for item in models]
	colors = [
		[0.1*(idx+1),0,0] for idx in range(len(obj_paths))]
	texture_paths = ['' for item in obj_paths]
	obj_paths += ['models/{}/textured_simple.obj'.format(obj) for obj in objs]
	texture_paths += ['models/{}/texture_map.png'.format(obj) for obj in objs]
	colors += [[0.3 + 0.1 * i,0,0] for i in range(len(objs))]

	cls_indexes = range(len(obj_paths)) #7
	renderer = YCBRenderer(width=width, height=height, render_marker=False, robot='panda_arm')
	robot = robot_kinematics('panda_arm')
	renderer.load_objects(obj_paths, texture_paths, colors)
	renderer.set_camera_default()
	renderer.set_projection_matrix(640, 480, 525, 525, 319.5, 239.5, 0.001, 1000)
	renderer.set_camera_default()
	renderer.set_light_pos(np.random.uniform(-0.5, 0.5, 3))
	image_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
	seg_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
	obj_name = '_'.join(classes[target].split('_')[1:])
	print 'output/%s_grasp_pose.txt'%obj_name
	rot = quat2mat(pose_grasp.orientation)
	joint_val = 5 #pose_grasp[:, -2]
	energy = 40 #pose_grasp[:, -1]
	pose_grasp = np.eye(4)
	pose_grasp[:3, 3] = pose_grasp.position
	pose_grasp[:3, :3] = quat2mat(rot)
	mkdir_if_missing('test_image')
	mkdir_if_missing('test_image/%s'%obj_name)
	pose_grasp[:, :3, 3] /= 1000 #scale the translation
	pos = [0, 0, 0]



	for idx in range(pose_grasp.shape[0]):
		target_pose = np.eye(4)
		pose = mat_poses[:, :, np.where(cls_indexes == target)]
		quat = np.random.randn(4)
		
		pose_scene = get_best_grasp(object_pose[target_idx])
		pose_scene += object_pose
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
		while len(sys.argv) > 3:
			renderer.render(cls_indexes, image_tensor, seg_tensor)
			image_tensor = image_tensor.flip(0)
			seg_tensor = seg_tensor.flip(0)
			frame = [image_tensor.cpu().numpy(), seg_tensor.cpu().numpy()]
			centers = renderer.get_centers()
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


height = 480
width = 640
video = sys.argv[1] #/home/liruiw/Projects/grasp/src/graspit_pose/script/YCB_Video/data/0040/000001-meta.mat
mat_file = 'YCB_Video/data/{}/000001-meta.mat'.format(video)
target = int(sys.argv[2]) #5
classes = ('__background__', '002_master_chef_can', '003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', \
                         '007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana', '019_pitcher_base', \
                         '021_bleach_cleanser', '024_bowl', '025_mug', '035_power_drill', '036_wood_block', '037_scissors', '040_large_marker', \
                         '051_large_clamp', '052_extra_large_clamp', '061_foam_brick')

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
print objs

# ros
def example_publish():
	rospy.init_node('example_publisher')
	rate = rospy.Rate(30) 
	pose_pub = rospy.Publisher('object_poses', PoseArray, queue_size=1) #add prefix
	target_pub = rospy.Publisher('grasp_target', Int32, queue_size=1)
	classes_pub = rospy.Publisher('grasp_classes', Int32MultiArray, queue_size=1)
	rospy.Subscriber('grasp_pose', Pose, call_back)
	poseArr = []
	class_msg = Int32MultiArray()
	class_msg.data = cls_indexes
	classes_pub.publish(class_msg)
	target_pub.publish(target)
	msgs = PoseArray()
	for i in range(cls_indexes.shape[0]):
		pose = Pose(object_pose[i][:3, 3], mat2quat(object_pose[i][:3, :3])) #w order?
		msgs.poses.append(pose)
	pose_pub.publish(msgs)
	rospy.sleep(0.01)
	rospy.spin()

if __name__ == '__main__':
    try:
        example_publish()
    except rospy.ROSInterruptException:
        pass