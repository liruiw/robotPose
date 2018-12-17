import numpy as np
from FKSolver import FKSolver
import os
from math import *
from numpy.linalg import inv

camera_extrinsics=np.array([[-0.211719, 0.97654, -0.0393032, 0.377451],[0.166697, -0.00354316, -0.986002, 0.374476],[-0.96301, -0.215307, -0.162036, 1.87315],[0,0, 0, 1]])
camera_intrinsics=np.array([[525, 0, 319.5],[ 0, 525, 239.5],[0, 0, 1]])
cur_path = os.path.abspath(os.path.dirname(__file__))
baxfk = FKSolver()


# print "Joint to Pose"
# joint_pose_1 = {'s0':-128, 's1':-82.2, 'e0':0, 'e1':45, 'w0':22.4, 'w1':-69, 'w2':0}
# joint_pose_2 = {'s0':50, 's1':50, 'e0':50, 'e1':50, 'w0':50, 'w1':50, 'w2':50}
# joint_pose_3 = {'s0':50, 's1':-30, 'e0':50, 'e1':20, 'w0':50, 'w1':60, 'w2':50}
# joint_pose_4 = {'s0':20, 's1':-30, 'e0':50, 'e1':46, 'w0':62, 'w1':24, 'w2':50}
# joint_pose_5 = {'s0':40, 's1':-25, 'e0':-102, 'e1':20, 'w0':50, 'w1':-60, 'w2':50}
# joint_pose_6 = {'s0':50, 's1':-30, 'e0':54, 'e1':120, 'w0':-170, 'w1':-89.9, 'w2':-26}
# joint_pose = [joint_pose_1,joint_pose_2,joint_pose_3,joint_pose_4,joint_pose_5,joint_pose_6]
# for pose in joint_pose:
#     pose = baxfk.solve_poses_from_joint(pose)
#     arm_test_image = cv2.imread(os.path.join(cur_path, '../../../test_data/%06d-color.png'%1))
#     poses = []
#     inv_poses = np.zeros([4,4,8])
#     inv_poses[:,:,0] = np.eye(4)
#     for i in range(7):
#         poses.append(pose[i][:3,:].copy())
#         inv_poses[:,:,i+1] = pose[i]
#     #arm_color, arm_depth = vtk_model.apply_transform(poses)
import scipy.io as sio
import cv2
import os.path
import random
def mkdir_if_missing(dst_dir):
    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)

def to4x4(T): 
    new_T = np.zeros([4,4])
    new_T[:3,:4] = T
    new_T[3,3] = 1
    return new_T    

def r2c(T_list):
    res = []
    for T in T_list:
        res.append((camera_extrinsics.dot(T))[:3,:])
    return res
def load_joints(joint): #expect a list
	return {'s0':joint[0], 's1':joint[1], 'e0':joint[2], 'e1':joint[3], 'w0':joint[4], 'w1':joint[5], 'w2':joint[6]}

from baxter_synthesizer import Camera_VTK
vtk_model = Camera_VTK(visualize=False) 
random.seed(233)
for i in range(1):
    # index = random.randint(0,87045)
    file = sio.loadmat('/local/nas/liruiw/arun_baxter/real_data/000001-meta.mat')
    pose_cam = file['poses']
    arm_test_image = cv2.imread('/local/nas/liruiw/arun_baxter/real_data/000001-color.png')
    pose_r = np.zeros([4,4,8])
    pose_r[:,:,0] = np.eye(4) #origin
    for i in range(7):
        pose_i = inv(camera_extrinsics).dot(to4x4(pose_cam[:,:,i+1])) #cam to r
        pose_r[:,:,i+1] = pose_i
    print "Pose to Joint"
    joints = baxfk.solve_joint_from_poses(pose_r) 
    #joints = [0,30,0,0,0,0,0]
    print 'joint angles:', joints
    joints = load_joints(joints)  
    pose = baxfk.solve_poses_from_joint(joints) #the original pose pose->joint->pose
    # for i in range(len(pose)):
    #     print pose[i]
    #perturbed_pose = baxfk.perturb_pose(pose_r) #call on pose -> joint + delta -> pose
    mkdir_if_missing('test_image')
    arm_color, arm_depth = vtk_model.apply_transform(r2c(pose))
    arm_test_image[arm_depth!=0] = arm_color[arm_depth!=0]
    cv2.imwrite( 'test_image/test_jp.png',arm_test_image)