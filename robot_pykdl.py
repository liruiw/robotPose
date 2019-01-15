#!/usr/bin/python

import numpy as np
import PyKDL
import scipy.io as sio
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat, mat2euler, euler2quat
from transforms3d.axangles import mat2axangle
import os
import numpy.random as random
from numpy.linalg import inv
from kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from ycb_renderer import YCBRenderer
import torch

def mkdir_if_missing(dst_dir):
    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)
def list2M(pose_list):
    poses = np.zeros([4,4,len(pose_list)])
    for idx, pose in enumerate(pose_list):
        poses[:,:,idx] = pose
    return poses
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
        if robot == 'panda_arm':
            cur_path = os.path.dirname(os.path.abspath(__file__))
            self._robot = URDF.from_xml_string(open(os.path.join(cur_path,'panda_arm.urdf'), 'r+').read())
            self._tip_link = 'panda_link7'
        else: #baxter right limb
            cur_path = os.path.dirname(os.path.abspath(__file__))
            self._robot = URDF.from_xml_string(open(os.path.join(cur_path,'baxter_base.urdf'), 'r+').read())
            self._tip_link = 'right_wrist' 
        self._kdl_tree = kdl_tree_from_urdf_model(self._robot)
        self._base_link = self._robot.get_root() #set it to base
         #we are not interested in gripper
        self._tip_frame = PyKDL.Frame()
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)
        print self._base_link
        self._joint_name, self._joint_limits, self._joint2tips, self._pose_0 = self.get_joint_info()
        print  self._joint_name,self._joint_limits
        # KDL Solvers not used for now
    
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
    
    def get_joint_info(self, print_kdl_chain=False):
        """
        Load joint limits from description file
        """
        robot_description = self._robot
        joint_limits = {}
        joints = []
        segment_joint2tip = []
        initial_pose = []
        for idx in xrange(self._arm_chain.getNrOfSegments()):
            segment = self._arm_chain.getSegment(idx)
            joint = segment.getJoint()
            joint_name = joint.getName().encode("utf-8") #get rid of unicode
            joint2tip = segment.getFrameToTip()
            pose_j2t = np.eye(4) 
            for i in range(3):
                for j in range(4):
                    pose_j2t[i,j] = joint2tip[i,j] #save fix joint to tip 
            segment_joint2tip.append(pose_j2t)
            pose_0 = segment.pose(0)
            pose_init = np.eye(4) 
            for i in range(3):
                for j in range(4):
                    pose_init[i,j] = pose_0[i,j]  #save fix initial angle pose
            initial_pose.append(pose_init)
            if print_kdl_chain:
                print '* ' + joint_name
            joints.append(joint_name)
        
        for joint in robot_description.joints:
            if joint.limit and joint.name in joints:
                joint_limits[joint.name] = [joint.limit.lower,joint.limit.upper]
        return joints, joint_limits, segment_joint2tip, initial_pose

    def solve_joint_from_poses(self, pose, base_link='right_arm_mount', base_pose=None): 
        """
        Input 4x4xn poses in robot coordinates, output list of joint angels in degrees.
        Joints before base link assumes to have joint angle 0, or a base pose can be provided
        """
        joint_values = [] 
        base = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
        if type(pose) == list:
            pose = list2M(pose)
        cur_pose = base_pose  
        if base_pose is None: # asssume all zero angles
            cur_pose = np.eye(4) 
            for k in range(base):
                cur_pose = cur_pose.dot(self._pose_0[k])   
        for idx in range(pose.shape[-1]):
            tip2tip = inv(cur_pose).dot(pose[:,:,idx])
            segment = self._arm_chain.getSegment(idx+base)
            pose_j2t = self._joint2tips[idx+base]           
            pose_joint = inv(pose_j2t).dot(tip2tip)
            #rotate_axis = segment.getJoint().JointAxis()
            axis, angle = mat2axangle(pose_joint[:3,:3]) 
            joint_values.append(angle)
            cur_pose = pose[:,:,idx]
        return rad2deg(np.array(joint_values)) 

    def solve_poses_from_joint(self, joint_values=None, base_link='right_arm_mount', base_pose=None):
        """
        Input joint angles in degrees, output poses list in robot coordinates 
        If a base pose is given, starts at that pose otherwise insert 0 degree joint pose at front
        """
        poses = []
        pose_deltas = []
        if self.check_joint_limits(joint_values, base_link=base_link):
            joint_values = deg2rad(joint_values)
            cur_pose = base_pose
            base = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
            if base_pose is None: # asssume all zero angles
                cur_pose = np.eye(4)
                for idx in xrange(base):     
                    pose_end = self._pose_0[idx]
                    cur_pose = cur_pose.dot(pose_end)                
            for idx in xrange(joint_values.shape[0]):     
                pose_ = self._arm_chain.getSegment(idx+base).pose(joint_values[idx])
                pose_end = np.eye(4)
                for i in range(3):
                    for j in range(4):
                        pose_end[i,j] = pose_[i,j]
                cur_pose = cur_pose.dot(pose_end)
                poses.append(cur_pose.copy())
                pose_deltas.append(pose_end.copy())

            return poses

    def perturb_pose(self, pose, base_link='right_arm_mount', scale=5, base_pose=None):
        """
        Perturb the initial pose based on joint angles and return the perturb joint angles
        """
        num = len(pose)
        joints_t = self.solve_joint_from_poses(pose, base_link, base_pose)
        joints_p = joints_t + scale * np.random.randn(num) 
        #joints_p = joints_t - scale # fix perturb
        while not self.check_joint_limits(joints_p,base_link):
            joints_p = joints_t + scale*np.random.randn(num)
        return self.solve_poses_from_joint(joints_p, base_link, base_pose), joints_p

    def get_pose_delta(self, pose_delta, base_link='right_arm_mount'):
        """
        Return new pose delta based on joint angles
        """
        new_pose_delta = pose_delta.copy()
        for i in range(pose_delta.shape[0]):
            pose = np.eye(4)

            pose[:3,:3] = quat2mat(pose_delta[i, 2:6])
            pose[:3, 3] = pose_delta[i, 6:]
            idx = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
            segment = self._arm_chain.getSegment(idx)
            pose_j2t = self._joint2tips[idx]  
            
            pose_joint = inv(pose_j2t).dot(pose)

            axis, angle = mat2axangle(pose_joint[:3,:3])  #regress the angle?
            pose_ = self._arm_chain.getSegment(idx).pose(angle) 
            for i in range(3):
                for j in range(4):
                    pose[i,j] = pose_[i,j]
            new_pose_delta[i, 2:6] = mat2quat(pose[:3,:3])
            new_pose_delta[i, 6:] = pose[:3, 3]
        return new_pose_delta[:, 2:6], new_pose_delta[:, 6:] #quaternion delta, translation

    def check_joint_limits(self, joint_values, base_link='right_arm_mount'):
        """
        Check the joint limits based on urdf
        """
        num = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
        joint_values = deg2rad(joint_values)
        for idx in range(joint_values.shape[0]):    
            joint_name = self._joint_name[num+idx]
            lower_bound_check = joint_values[idx] >= self._joint_limits[joint_name][0]
            upper_bound_check = joint_values[idx] <= self._joint_limits[joint_name][1]
            if not (lower_bound_check and upper_bound_check):
                print "{} joint limits exceeded! angle: {}".format(joint_name, joint_values[idx])
                return False
        return True

    def gen_rand_pose(self, base_link='right_arm_mount', base_pose=None):
        """
        Generate random poses given base link or base pose
        """
        joint_values = []
        margin = 0.1
        num = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
        for idx in range(num, self._arm_chain.getNrOfSegments()):
            joint_name = self._joint_name[idx]
            ub = min(self._joint_limits[joint_name][1] - margin, 3.14) #joint 6 has limit = 3.8 which causes problem in solve inv
            lb = max(self._joint_limits[joint_name][0] + margin, -3.14)
            joint_values.append(np.random.uniform(lb, ub))
            #joint_values[-1] = 0 # fix initial pose
        return self.solve_poses_from_joint(rad2deg(np.array(joint_values)),base_link, base_pose=base_pose), joint_values

def to4x4(T): 
    new_T = np.eye(4)
    new_T[:3,:4] = T
    return new_T
    
def r2c(T_list):
    res = []
    for T in T_list:
        res.append((camera_extrinsics.dot(T))[:3,:])
    return res

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, default='baxter', help='Robot Name')
    parser.add_argument('--test', type=str, default='all', help='Robot Name')
    args = parser.parse_args()
    robot = robot_kinematics(args.robot)
    width = 640 #800
    height = 480 #600

    #from robot_synthesizer import Camera_VTK
    import cv2
    mkdir_if_missing('test_image')
    print 'robot name', args.robot
    renderer = YCBRenderer(width=width, height=height, render_marker=False)
    if args.robot == 'panda_arm':
        models = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7']
        base_link = 'panda_link0'
        if args.test == 'middle':
            base_link = 'panda_link3'
        base_idx = 0
        if base_link[-5:] in models:
            base_idx = models.index(base_link[-5:])+1
            models = models[base_idx:]
        obj_paths = [
            '{}_models/{}.DAE'.format(args.robot,item) for item in models]
        colors = [
            [0.1*(idx+1),0,0] for idx in range(len(models))]
        texture_paths = ['' for item in models]
        cls_indexes = range(7 - base_idx) #7
    elif args.robot == 'baxter':
        models = ['S0', 'S1', 'E0', 'E1', 'W0', 'W1', 'W2']
        base_idx = 0
        base_link = 'right_arm_mount'
        if args.test[:6] == 'middle':
            base_link = 'right_upper_elbow'
            base_idx = 3
        models = models[base_idx:]
        obj_paths = [
            '{}_models/{}.DAE'.format(args.robot,item) for item in models]
        colors = [
            [0.1*(idx+1),0,0] for idx in range(len(models))]
        texture_paths = ['' for item in models]
        cls_indexes = range(7 - base_idx) #7

    renderer.load_objects(obj_paths, texture_paths, colors)
    
    renderer.set_camera_default()
    renderer.set_projection_matrix(640, 480, 525, 525, 319.5, 239.5, 0.001, 1000)
    renderer.set_light_pos([0, 0, 1])
    image_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
    seg_tensor = torch.cuda.FloatTensor(height, width, 4).detach()

    for index in range(5):
        file = sio.loadmat('sample_data/%06d-meta.mat'%(index % 5))
        #panda and baxter have dof 7, we are interested in 6
        arm_test_image = cv2.imread('sample_data/%06d-color.png'%(index % 5))
        if args.robot == 'panda_arm':  #correspond with move_arm
            initial_pt = np.array([0,0,1])     
            r = np.zeros(3)  
            r[0] = np.random.uniform(low=-np.pi/2, high=np.pi/2)
            r[2] = np.random.uniform(low=-np.pi, high=np.pi)
            rot = euler2mat(*r)
            pos = np.matmul(rot, initial_pt)
            pos = (pos / np.linalg.norm(pos)) * np.random.uniform(low=2.3, high=2.5)
            pos = np.array([2.6, -1.8, 1.2])        
            renderer.set_camera(pos, 2*pos, [0,0,-1])
            renderer.set_light_pos(pos + np.random.uniform(-0.5, 0.5, 3))
            intensity = np.random.uniform(0.8, 2)
            light_color = intensity * np.random.uniform(0.9, 1.1, 3)
            renderer.set_light_color(light_color)  

        else:
            pose_cam = file['poses']
            pose_r = np.zeros([4,4,7]) #assume the poses before arm has all 0 joint angles
            camera_extrinsics=np.array([[-0.211719, 0.97654, -0.0393032, 0.377451],[0.166697, -0.00354316, -0.986002, 0.374476],[-0.96301, -0.215307, -0.162036, 1.87315],[0,0, 0, 1]])
            for i in range(7):
                pose_i = inv(camera_extrinsics).dot(to4x4(pose_cam[:,:,i+1])) #cam to r
                pose_r[:,:,i] = pose_i       
            
            joints = robot.solve_joint_from_poses(pose_r, 'right_arm_mount')
            renderer.V = camera_extrinsics
            poses_p = robot.solve_poses_from_joint(joints[base_idx:],base_link)
            if base_idx > 0:
                print 'base pose test'
                print joints[base_idx:]
                poses_p = robot.solve_poses_from_joint(joints[base_idx:],base_link,base_pose=pose_r[:,:,base_idx-1])
                joints_p = robot.solve_joint_from_poses(poses_p,base_link,base_pose=pose_r[:,:,base_idx-1])
                print joints_p
        if args.robot != 'baxter' or args.test[-3:] != 'fix':
            poses_p, joint = robot.gen_rand_pose(base_link)[:len(cls_indexes)]
            joint_test = robot.solve_poses_from_joint(np.array(joint), base_link, base_pose=np.eye(4))
            print 'joint test', joint  
            print robot.solve_joint_from_poses(joint_test, base_link, base_pose=np.eye(4))         
            poses_p, perturb = robot.perturb_pose(poses_p, base_link, base_pose=np.eye(4))
            print  'perturb test', perturb
        poses = []
        for i in range(len(poses_p)):
            pose_i = poses_p[i]
            rot = mat2quat(pose_i[:3,:3])
            trans = pose_i[:3,3]
            poses.append(np.hstack((trans,rot))) 
        #rendering test
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
        arm_test_image[mask[:,:,2]!=0] = image[mask[:,:,2]!=0] #red channel
        cv2.imwrite( 'test_image/%06d-color.png'%index,arm_test_image)

camera_intrinsics=np.array([[525, 0, 319.5],[ 0, 525, 239.5],[0, 0, 1]])

if __name__ == "__main__":
    main()
