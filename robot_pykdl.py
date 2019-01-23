#!/usr/bin/python

import numpy as np
import PyKDL
import scipy.io as sio
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat, mat2euler, euler2quat
from transforms3d.axangles import mat2axangle
import os
import sys
import numpy.random as random
from numpy.linalg import inv
from kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from ycb_renderer import YCBRenderer
import torch
np.random.seed(233)
def rotZ(rotz):
    RotZ = np.matrix([[np.cos(rotz), -np.sin(rotz), 0, 0], 
                  [np.sin(rotz), np.cos(rotz), 0, 0], 
                  [0, 0, 1, 0], 
                  [0, 0, 0, 1]])
    return RotZ

def to4x4(T): 
    new_T = np.eye(4)
    new_T[:3,:4] = T
    return new_T

def mkdir_if_missing(dst_dir):
    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)

def list2M(pose_list):
    poses = np.zeros([4,4,len(pose_list)])
    for idx, pose in enumerate(pose_list):
        poses[:,:,idx] = pose
    return poses

def M2list(pose):
    pose_list = []
    for idx in range(pose.shape[-1]):
        pose_list.append(pose[:,:,idx])
    return pose_list

def deg2rad(deg):
    return deg/180.0*np.pi

def rad2deg(rad):
    return rad/np.pi*180

def pose2np(pose_kdl):
    pose = np.eye(4)
    for i in range(3):
        for j in range(4):
            pose[i,j] = pose_kdl[i,j]  #save fix initial angle pose   
    return pose 

class robot_kinematics(object):
    """
    Robot Kinematics with PyKDL
    """
    def __init__(self, robot):
        #self._baxter = URDF.from_parameter_server(key='robot_description')
        if robot == 'panda_arm':
            cur_path = os.path.dirname(os.path.abspath(__file__))
            self._robot = URDF.from_xml_string(open(os.path.join(cur_path,'panda_arm_hand.urdf'), 'r+').read())
            self._tip_link = 'panda_hand'
            self._end_effector = ['panda_leftfinger', 'panda_rightfinger']
        else: #baxter right limb
            cur_path = os.path.dirname(os.path.abspath(__file__))
            self._robot = URDF.from_xml_string(open(os.path.join(cur_path,'baxter_base.urdf'), 'r+').read())
            self._tip_link = 'right_wrist' 
            self._end_effector = []
        self._name = robot
        self._kdl_tree = kdl_tree_from_urdf_model(self._robot)
        self._base_link = self._robot.get_root() #set it to base
         #we are not interested in gripper
        self._tip_frame = PyKDL.Frame()
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)
        self._empty_link_ids = []
        self.center_offset = self.load_offset()
        self._links = self.load_end_effector()
        self._joint_name, self._joint_limits, self._joint2tips, self._pose_0 = self.get_joint_info()
        self.prepare_virtual_links()
        
        print 'robot name {} with base link {}'.format(self._name, self._base_link)
        print self._joint_name, self._joint_limits
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
    
    def load_end_effector(self):
        links = []
        for idx in range(self._arm_chain.getNrOfSegments()):
            links.append(self._arm_chain.getSegment(idx))
        for end_effector in self._end_effector:
            chain = self._kdl_tree.getChain(self._tip_link, end_effector)
            for idx in range(chain.getNrOfSegments()):
                links.append(chain.getSegment(idx))
        return links
       
    def prepare_virtual_links(self):
        # remove the virtual link 8 and combine the joint poses with hand 
        # could also combine the link in urdf parsing process
        for i, empty_link in enumerate(self._empty_link_ids):
            idx = empty_link - i
            del self._links[idx]
            hand_pose = self._joint2tips[idx].dot(self._joint2tips[idx+1])
            self._joint2tips[idx+1] = hand_pose
            del self._joint2tips[idx]
            self._pose_0[idx+1] = hand_pose #the same for fixed joint
            del self._pose_0[idx]
            del self._joint_name[idx]
    
    def get_joint_info(self, print_kdl_chain=False):
        """
        Load joint limits from description file
        """
        robot_description = self._robot
        joint_limits = {}
        joints = []
        segment_joint2tip = []
        initial_pose = []
        for segment in self._links:
            joint = segment.getJoint()
            joint_name = joint.getName().encode("utf-8") #get rid of unicode
            segment_joint2tip.append(pose2np(segment.getFrameToTip())) #save fix initial angle pose
            initial_pose.append(pose2np(segment.pose(0)))
            if print_kdl_chain:
                print '* ' + joint_name
            joints.append(joint_name)

        for joint in robot_description.joints:
            if joint.name in joints and joint.limit:
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
            segment = self._links[idx + base]
            pose_j2t = self._joint2tips[idx + base]           
            pose_joint = inv(pose_j2t).dot(tip2tip)
            #rotate_axis = segment.getJoint().JointAxis()
            axis, angle = mat2axangle(pose_joint[:3,:3]) 
            joint_values.append(angle)
            cur_pose = pose[:, :, idx]
            if idx + base == 8 and len(self._end_effector) > 0: #right finger reuse hand pose
                cur_pose = pose[:,:,-3].copy()
        return rad2deg(np.array(joint_values)) 

    def solve_poses_from_joint(self, joint_values=None, base_link='right_arm_mount', base_pose=None):
        """
        Input joint angles in degrees, output poses list in robot coordinates 
        If a base pose is given, starts at that pose otherwise insert 0 degree joint pose at front
        """
        poses = []
        if self.check_joint_limits(joint_values, base_link=base_link):
            joint_values = deg2rad(joint_values)
            cur_pose = base_pose
            base = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
            if base_pose is None: # asssume all zero angles
                cur_pose = np.eye(4)
                for idx in xrange(base):     
                    pose_end = self._pose_0[idx].copy()
                    cur_pose = cur_pose.dot(pose_end)

            for idx in xrange(joint_values.shape[0]): 
                cur_pose = cur_pose.dot(pose2np(self._links[idx + base].pose(joint_values[idx])))
                if idx + base == 7 and len(self._end_effector) > 0: # fixed combined joint 8 and hand joint
                    if base == 7:
                        cur_pose = base_pose.dot(self._joint2tips[idx + base]) #fix pose anyways
                    else:
                        cur_pose = poses[-1].dot(self._joint2tips[idx + base])
                poses.append(cur_pose.copy())
                if idx + base == 8 and len(self._end_effector) > 0: #right finger reuse hand pose
                    cur_pose = poses[-2].copy()
            return poses
        print 'invalid joint to solve poses'

    def perturb_pose(self, pose, base_link='right_arm_mount', scale=5, base_pose=None, center_offset=False):
        """
        Perturb the initial pose based on joint angles and return the perturb joint angles
        """
        if center_offset:
            pose = self.offset_pose_center(pose, dir='on', base_link=base_link)
        num = len(pose)
        base = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()

        joints_t = self.solve_joint_from_poses(pose, base_link, base_pose)
        joints_p = joints_t + scale * np.random.randn(num) 
        joints_p =  self.sample_ef(joints_p, base+num)
        while not self.check_joint_limits(joints_p, base_link):
            joints_p = joints_t + scale*np.random.randn(num)
            joints_p = self.sample_ef(joints_p, base+num)
        pose = self.solve_poses_from_joint(joints_p, base_link, base_pose)
        if center_offset:
            pose = self.offset_pose_center(pose, dir='off', base_link=base_link)  
        return pose, joints_p

    def sample_ef(self, joints, size):
        # perturb prismatic joint for panda
        if size == len(self._links) and len(self._end_effector) > 0:
            joints[-1] = np.random.uniform(0, 2.29); 
            joints[-2] = np.random.uniform(0, 2.29);  #0.04*180/pi
        return joints          
    
    def check_joint_limits(self, joint_values, base_link='right_arm_mount'):
        """
        Check the joint limits based on urdf
        """
        num = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
        joint_values = deg2rad(joint_values)
        for idx in range(joint_values.shape[0]):  
            joint_name = self._joint_name[num + idx]
            if joint_name in self._joint_limits:
                lower_bound_check = joint_values[idx] >= self._joint_limits[joint_name][0]
                upper_bound_check = joint_values[idx] <= self._joint_limits[joint_name][1]
                if not (lower_bound_check and upper_bound_check):
                    print "{} joint limits exceeded! value: {}".format(joint_name, joint_values[idx])
                    return False
        return True

    def gen_rand_pose(self, base_link='right_arm_mount', base_pose=None):
        """
        Generate random poses given base link or base pose
        """
        joint_values = []
        margin = 0.1 # 0.1 
        num = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
        for idx in range(num, len(self._links)):
            joint_name = self._joint_name[idx]
            if joint_name in self._joint_limits:
                ub = min(self._joint_limits[joint_name][1] - margin, 3.) #joint 6 has limit = 3.8 which causes problem in solve inv
                lb = max(self._joint_limits[joint_name][0] + margin, -3.)
                joint_values.append(np.random.uniform(lb, ub))
            else: # fix joint
                joint_values.append(0)
        joint_values = rad2deg(np.array(joint_values))
        joint_values = self.sample_ef(joint_values, num + len(joint_values))
        return self.solve_poses_from_joint(joint_values, base_link, base_pose=base_pose), joint_values
    
    def load_offset(self):
        # load the offset from the presaved txt file (fixed link and repeated finger offset are added)
        cur_path = os.path.abspath(os.path.dirname(__file__))
        offset_file = os.path.join(cur_path, self._name + '_models', 'center_offset.txt')
        offset = np.loadtxt(offset_file).astype(np.float32)
        offset_list = []
        for i in range(offset.shape[0]):
            if not offset[i, :].any():
               self._empty_link_ids.append(i)   #mark the empty link id and skip
               continue
            offset_pose = np.eye(4)
            offset_pose[:3, 3] = offset[i, :]
            offset_list.append(offset_pose)
        #extent = max - min, center = (max + min)/2 (3D bounding box)
        return offset_list
        
    def offset_pose_center(self, pose, dir='off', base_link='right_arm_mount'): 
        """
        Off means from original pose to the centered pose in model coordinate.
        """ 
        base_idx = self._kdl_tree.getChain(self._base_link, base_link).getNrOfSegments()
        input_list = False  
        if type(pose) == list:
            input_list = True
            pose = list2M(pose)
        for i in range(pose.shape[-1]):
            offset_pose = self.center_offset[base_idx + i].copy()
            if dir == 'on':     
                offset_pose[:3, 3] *= -1 
            if base_idx + i == 9: # right finger has origin flipped in urdf :(
                offset_pose[:3, :3] = rotZ(np.pi)[:3, :3]  
            pose[:, :, i] = pose[:, :, i].dot(offset_pose)
        
        if input_list:
            return M2list(pose)
        return pose
    
    def get_link_name(self, name=None, link_id=None):
        if name:
            return name.strip().split('_')[-1]
        elif link_id:
            return self._arm_chain.getSegment(link_id).getName()
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
    renderer = YCBRenderer(width=width, height=height, render_marker=False, robot=args.robot)
    if args.robot == 'panda_arm':
        models = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'hand', 'finger', 'finger']
        base_link = 'panda_link0'
        if args.test == 'middle':
            base_link = 'panda_link3'
        base_idx = 0
        name = base_link.strip().split('_')[-1]
        if name in models:
            base_idx = models.index(name) + 1 #take the link name
        obj_paths = [
            '{}_models/{}.DAE'.format(args.robot,item) for item in models]
        colors = [
            [0.1*(idx+1),0,0] for idx in range(len(models))]
        texture_paths = ['' for item in models]
        cls_indexes = range(base_idx, len(models)) #7
    elif args.robot == 'baxter':
        models = ['S0', 'S1', 'E0', 'E1', 'W0', 'W1', 'W2']
        base_idx = 0
        base_link = 'right_arm_mount'
        if args.test[:6] == 'middle':
            base_link = 'right_upper_elbow'
            base_idx = 3
        #models = models[base_idx:]
        obj_paths = [
            '{}_models/{}.DAE'.format(args.robot,item) for item in models]
        colors = [
            [0.1*(idx+1),0,0] for idx in range(len(models))]
        texture_paths = ['' for item in models]
        cls_indexes = range(base_idx, len(models)) #7

    renderer.load_objects(obj_paths, texture_paths, colors)
    renderer.set_camera_default()
    renderer.set_projection_matrix(640, 480, 525, 525, 319.5, 239.5, 0.001, 1000)
    renderer.set_light_pos([0, 0, 1])
    image_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
    seg_tensor = torch.cuda.FloatTensor(height, width, 4).detach()

    for index in range(10):
        file = sio.loadmat('sample_data/%06d-meta.mat'%(index % 5))
        arm_test_image = cv2.imread('sample_data/%06d-color.png'%(index % 5))
        if args.robot == 'panda_arm':  #correspond with move_arm
            initial_pt = np.array([0,0,1])     
            r = np.zeros(3)  
            r[0] = np.random.uniform(low=-np.pi/2, high=np.pi/2)
            r[2] = np.random.uniform(low=-np.pi, high=np.pi)
            rot = euler2mat(*r)
            pos = np.matmul(rot, initial_pt)
            pos = (pos / np.linalg.norm(pos)) * np.random.uniform(low=2.3, high=2.5)
            pos = np.array([0.6, -1.8, 1.2])        
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
                print 'middle base test ==================='
                poses_p = robot.solve_poses_from_joint(joints[base_idx:],base_link,base_pose=pose_r[:,:,base_idx-1])
                joints_p = robot.solve_joint_from_poses(poses_p, base_link,base_pose=pose_r[:,:,base_idx-1])   
                print '======================'
            poses_p = robot.offset_pose_center(poses_p, dir='off', base_link=base_link) #original -> center pose
        
        if args.robot != 'baxter':

            poses_p, joint = robot.gen_rand_pose(base_link)[:len(cls_indexes)]
            joint_test = robot.solve_poses_from_joint(np.array(joint), base_link, base_pose=np.eye(4))
            print 'joint test ==================='
            print joint
            print robot.solve_joint_from_poses(joint_test, base_link, base_pose=np.eye(4))
            print '======================'

            # print 'mesh center test ==================='
            poses_p = robot.offset_pose_center(poses_p, dir='off', base_link=base_link) 
            poses_p = robot.offset_pose_center(poses_p, dir='on', base_link=base_link)  #center -> original pose
            print deg2rad(robot.solve_joint_from_poses(poses_p, base_link, base_pose=np.eye(4)))

            poses_p = robot.offset_pose_center(poses_p, dir='off', base_link=base_link)  #original -> center pose
            poses_p = robot.offset_pose_center(poses_p, dir='on', base_link=base_link) 
            print deg2rad(robot.solve_joint_from_poses(poses_p, base_link, base_pose=np.eye(4)))

            print 'perturb test ==================='
            poses_p, perturb = robot.perturb_pose(poses_p, base_link, base_pose=np.eye(4))
            poses_p = robot.offset_pose_center(poses_p, dir='off', base_link=base_link)
            poses_p = robot.offset_pose_center(poses_p, dir='on', base_link=base_link)
            poses_p, perturb = robot.perturb_pose(poses_p, base_link, base_pose=np.eye(4), center_offset=True)
            # #poses_p = robot.offset_pose_center(poses_p, dir='on', base_link=base_link)
            # print perturb
            print '======================'         
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
