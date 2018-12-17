#!/usr/bin/python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import PyKDL
import scipy.io as sio
from transforms3d.quaternions import quat2mat
from transforms3d.axangles import mat2axangle
import os
import numpy.random as random
from numpy.linalg import inv, norm
from kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF

def mkdir_if_missing(dst_dir):
    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)

def deg2rad(deg):
    return deg/180.0*np.pi

def rad2deg(rad):
    return rad/np.pi*180

class robot_kinematics(object):
    """
    Robot Kinematics with PyKDL
    """
    def __init__(self, robot):
        #self._baxter = URDF.from_parameter_server(key='robot_description')

        if robot == 'panda':
            self._robot = URDF.from_xml_string(open('panda_arm.urdf', 'r+').read())
            self._base_link = robot +'_link0'
            self._tip_link = robot +'_link7' #hard coded
        else: #baxter right limb
            self._robot = URDF.from_xml_string(open('baxter_base.urdf', 'r+').read())
            self._base_link = 'base'
            self._tip_link = 'right_wrist' 
        self._kdl_tree = kdl_tree_from_urdf_model(self._robot)
        self._base_link = self._robot.get_root() #set it to base
         #we are not interested in gripper
        self._tip_frame = PyKDL.Frame()
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)
        self._joint_names = self.get_kdl_chain(False)
        # KDL Solvers not used for now
        # self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._arm_chain)
        # self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._arm_chain)
        # self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        # self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_chain,
        #                                            self._fk_p_kdl,
        #                                            self._ik_v_kdl)
        # self._jac_kdl = PyKDL.ChainJntToJacSolver(self._arm_chain)
        # self._dyn_kdl = PyKDL.ChainDynParam(self._arm_chain,
        #                                     PyKDL.Vector.Zero())

    def print_robot_description(self):
        nf_joints = 0
        for j in self._robot.joints:
            if j.type != 'fixed':
                nf_joints += 1
        print "URDF non-fixed joints: %d;" % nf_joints
        print "URDF total joints: %d" % len(self._robot.joints)
        print "URDF links: %d" % len(self._robot.links)
        print "KDL joints: %d" % self._kdl_tree.getNrOfJoints()
        print "KDL segments: %d" % self._kdl_tree.getNrOfSegments()

    def get_kdl_chain(self, print_kdl_chain=False):
        joints = [] #initial two angles
        for idx in xrange(self._arm_chain.getNrOfSegments()):
            joint_name = self._arm_chain.getSegment(idx).getJoint().getName().encode("utf-8") #get rid of unicode
            if print_kdl_chain:
                print '* ' + joint_name
            joints.append(joint_name)
        return joints

    def solve_joint_from_poses(self, pose, base_link='right_arm_mount'): #4x4x8
        joint_values = [] 
        num = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
        poses = np.zeros([4,4,pose.shape[-1]+num])
        for k in range(num):
            poses[:,:,k] = np.eye(4)
            pose_ = self._arm_chain.getSegment(k).pose(0) #assume no angle
            for i in range(3):
                for j in range(4):
                    poses[i,j,k] = pose_[i,j]
        poses[:,:,num:] = pose
        angle = 0
        cur_pose = np.eye(4)
        for idx in range(poses.shape[-1]):
            tip2tip = inv(cur_pose).dot(poses[:,:,idx])
            segment = self._arm_chain.getSegment(idx)
            joint2tip = segment.getFrameToTip()
            pose_j2t = np.eye(4) 
            for i in range(3):
                for j in range(4):
                    pose_j2t[i,j] = joint2tip[i,j]    
            #pose_joint = inv(tip2tip).dot(pose_j2t)
            pose_joint = inv(pose_j2t).dot(tip2tip)
            rotate_axis = segment.getJoint().JointAxis()
            axis, angle = mat2axangle(pose_joint[:3,:3]) #seems ambiguous but correct
            joint_values.append(angle)
            cur_pose = poses[:,:,idx]
        return rad2deg(np.array(joint_values[num:])) 

    def solve_poses_from_joint(self,joint_values=None,base_link='right_arm_mount'):
        poses = []

        num = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
        joint_values = np.insert(joint_values,0,np.zeros(num)) # base to the interested joint
        joint_values = deg2rad(joint_values)
        cur_pose = np.eye(4)
        #print self._arm_chain.getNrOfSegments()
        for idx in xrange(self._arm_chain.getNrOfSegments()):
            pose_ = self._arm_chain.getSegment(idx).pose(joint_values[idx])
            pose_end = np.eye(4)
            for i in range(3):
                for j in range(4):
                    pose_end[i,j] = pose_[i,j]
            cur_pose = cur_pose.dot(pose_end)
            poses.append(cur_pose.copy())
        return poses[num:]  #we don't need the first two


camera_extrinsics=np.array([[-0.211719, 0.97654, -0.0393032, 0.377451],[0.166697, -0.00354316, -0.986002, 0.374476],[-0.96301, -0.215307, -0.162036, 1.87315],[0,0, 0, 1]])
camera_intrinsics=np.array([[525, 0, 319.5],[ 0, 525, 239.5],[0, 0, 1]])
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

def perpendicular(v):
    if (v[0]==0) and (v[1]==0):
        if (v[2]==0):
            raise ValueError('zero vector')
        return np.array([0, 1, 0])
    return np.array([-v[1], v[0], 0])

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, default='baxter', help='Robot Name')
    args = parser.parse_args()
    robot = robot_kinematics(args.robot)
    from baxter_synthesizer import Camera_VTK
    import cv2
    vtk_model = Camera_VTK(visualize=False)
    mkdir_if_missing('test_image')
    base_link='right_arm_mount'
    print 'robot name', args.robot
    if args.robot == 'panda':
        base_link='panda_link0'

    for index in range(5):
        file = sio.loadmat('sample_data/%06d-meta.mat'%index)
        #poses = robot.solve_poses_from_joint(np.array([ 0, 30, 15, 45,0,25, 0])) #panda and baxter have dof 7, we are interested in 6
        pose_cam = file['poses']
        arm_test_image = cv2.imread('sample_data/%06d-color.png'%index)
        pose_r = np.zeros([4,4,7]) #assume the poses before arm has all 0 joint angles
        for i in range(7):
            pose_i = inv(camera_extrinsics).dot(to4x4(pose_cam[:,:,i+1])) #cam to r
            pose_r[:,:,i] = pose_i
        #pose_r = robot.solve_poses_from_joint(np.array([0,0,0,0,0,0,0]))
        joints = robot.solve_joint_from_poses(pose_r,base_link) 
        poses = robot.solve_poses_from_joint(joints,base_link) 
        arm_test_image = cv2.imread('sample_data/%06d-color.png'%index)
        arm_color, arm_depth = vtk_model.apply_transform(r2c(poses))
        arm_test_image[arm_depth!=0] = arm_color[arm_depth!=0]
        cv2.imwrite( 'test_image/%06d-color.png'%index,arm_test_image)

if __name__ == "__main__":
    main()
