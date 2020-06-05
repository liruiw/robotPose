#!/usr/bin/python
import numpy as np
import PyKDL
import scipy.io as sio
import _init_paths
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
import cv2
import argparse
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
    if type(deg) is list:
        return [x/180.0*np.pi for x in deg]
    return deg/180.0*np.pi

def rad2deg(rad):
    if type(rad) is list:
        return [x/np.pi*180 for x in rad]
    return rad/np.pi*180

def pose2np(pose_kdl):
    pose = np.eye(4)
    for i in range(3):
        for j in range(4):
            pose[i,j] = pose_kdl[i,j]  #save fix initial angle pose   
    return pose 

class robot_kinematics(object):
    """
    Robot Kinematics built with PyKDL.
    It's mainly used to train a vision system for 7-dof robot arm with end effectors.
    """
    def __init__(self, robot, base_link=None, tip_link=None):
        cur_path = os.path.dirname(os.path.abspath(__file__))
        if robot == 'panda_arm':
            self._robot = URDF.from_xml_string(open(os.path.join(cur_path,\
            '{}_models'.format(robot), 'panda_arm_hand.urdf'), 'r+').read()) 
        elif robot == 'baxter': 
            self._robot = URDF.from_xml_string(open(os.path.join(cur_path,\
            '{}_models'.format(robot),  'baxter_arm_hand.urdf'), 'r+').read())
        
        self._name = robot
        self._kdl_tree, ef_info = kdl_tree_from_urdf_model(self._robot)
        if base_link is None:
            self._tip_link = ef_info.keys()[0]
            self._base_link = self._robot.get_root()
        else:
            self._base_link = base_link
            self._tip_link = tip_link
        self._end_effector = ef_info[self._tip_link]
        self._ef_num = len(self._end_effector)
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)
        self._num_jnts = self._arm_chain.getNrOfJoints()
        self._empty_link_ids = []
        self._links, self._link_names = self.get_link_info()
        self.center_offset = self.load_offset()
        
        self._joint_name, self._joint_limits, self._joint2tips, self._pose_0 \
                                      = self.get_joint_info()
        self.prepare_virtual_links()
        
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._arm_chain) # 
        self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._arm_chain)
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_chain, 
                                                    self._fk_p_kdl,
                                                    self._ik_v_kdl) 
        print('robot name {} with base link {}'.format(self._name, self._base_link))
        print self._joint_name, self._joint_limits
        
    def print_robot_description(self):
        nf_joints = 0
        for j in self._robot.joints:
            if j.type != 'fixed':
                nf_joints += 1
        print("URDF non-fixed joints: %d;" % nf_joints)
        print("URDF total joints: %d" % len(self._robot.joints))
        print("URDF links: %d" % len(self._robot.links))
        print("KDL joints: %d" % self._kdl_tree.getNrOfJoints())
        print("KDL segments: %d" % self._kdl_tree.getNrOfSegments())
    
    def get_link_info(self):
        links = []; link_names = [self._base_link];
        for idx in range(self._arm_chain.getNrOfSegments()):
            links.append(self._arm_chain.getSegment(idx))
            link_names.append(self._arm_chain.getSegment(idx).getName().encode("utf-8"))
        for end_effector in self._end_effector:
            chain = self._kdl_tree.getChain(self._tip_link, end_effector)
            for idx in range(chain.getNrOfSegments()):
                links.append(chain.getSegment(idx))
                link_names.append(chain.getSegment(idx).getName().encode("utf-8"))
        return links, link_names
       
    def prepare_virtual_links(self):
        # remove the virtual link 8 and combine the joint poses with hand 
        # could also combine the link in urdf parsing process
        for i, empty_link in enumerate(self._empty_link_ids):
            idx = empty_link - i
            del self._links[idx]
            del self._link_names[idx + 1]
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
            segment_joint2tip.append(pose2np(segment.getFrameToTip()))
            initial_pose.append(pose2np(segment.pose(0)))
            if print_kdl_chain:
                print('* ' + joint_name)
            joints.append(joint_name)

        for joint in robot_description.joints:
            if joint.name in joints and joint.limit:
                joint_limits[joint.name] = [joint.limit.lower,joint.limit.upper]
                index = joints.index(joint.name)
        return joints, joint_limits, segment_joint2tip, initial_pose

    def solve_joint_from_poses(self, pose, base_link='right_arm_mount', base_pose=None): 
        """
        Input 4x4xn poses in robot coordinates, output list of joint angels in degrees.
        Joints before base link assumes to have joint angle 0, or a base pose can be provided
        """
        joint_values = [] 
        base =  self._get_base_idx_shifted(base_link)
        if type(pose) == list:
            pose = list2M(pose)
        cur_pose = base_pose  
        if base_pose is None: # asssume all zero angles
            cur_pose = np.eye(4) 
            for k in range(base):
                cur_pose = cur_pose.dot(self._pose_0[k])

        for idx in range(pose.shape[-1]):
            tip2tip = inv(cur_pose).dot(pose[:,:,idx]) 
            pose_j2t = self._joint2tips[idx + base]           
            pose_joint = inv(pose_j2t).dot(tip2tip)
            #rotate_axis = segment.getJoint().JointAxis()
            axis, angle = mat2axangle(pose_joint[:3,:3]) 
            joint_values.append(angle)
            cur_pose = pose[:, :, idx]
            if idx + base >= len(self._links) - self._ef_num: # for all links attached with hand
                cur_pose = pose[:,:,-self._ef_num - 1].copy()
        return rad2deg(np.array(joint_values)) 

    def solve_poses_from_joint(self, joint_values=None, base_link='right_arm_mount', base_pose=None):
        """
        Input joint angles in degrees, output poses list in robot coordinates 
        If a base pose is given, starts at that pose otherwise insert 0 degree joint pose at front
        """
        poses = []
        joint_values = deg2rad(joint_values)
        cur_pose = base_pose
        base = self._get_base_idx_shifted(base_link)
        if base_pose is None: # asssume all zero angles
            cur_pose = np.eye(4)
            for idx in xrange(base):     
                pose_end = self._pose_0[idx].copy()
                cur_pose = cur_pose.dot(pose_end)

        for idx in xrange(joint_values.shape[0]): 
            cur_pose = cur_pose.dot(pose2np(self._links[idx + base].pose(joint_values[idx])))
            poses.append(cur_pose.copy())
        hand_pose = np.eye(4)
        hand_idx = len(self._links) - (self._ef_num + 1)
        if len(poses) > hand_idx: 
            hand_pose = poses[hand_idx - base - 1].dot(self._joint2tips[hand_idx])
            poses[hand_idx - base] = hand_pose.copy()
        if joint_values.shape[0] + base == len(self._links) and len(self._end_effector) > 0: #for hand and finger   
            if base_pose is not None and base == hand_idx: # link7
                hand_pose = base_pose.dot(self._joint2tips[hand_idx])
                poses[hand_idx] = hand_pose.copy()
            elif base_pose is not None and base ==  hand_idx + 1: # hand
                hand_pose = base_pose
            for i in range(1, self._ef_num + 1):
                poses[-i] = hand_pose.dot(pose2np(self._links[-i].pose(joint_values[-i]))).copy()
        return poses

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
        joints_p = self.sample_ef(joints_p, base+num)
        while not self.check_joint_limits(joints_p, base_link):
            joints_p = joints_t + scale*np.random.randn(num)
            joints_p = self.sample_ef(joints_p, base+num)
        pose = self.solve_poses_from_joint(joints_p, base_link, base_pose)
        if center_offset:
            pose = self.offset_pose_center(pose, dir='off', base_link=base_link)  
        return pose, joints_p

    def sample_ef(self, joints, size):
        # perturb prismatic joint for panda
        if size == len(self._links):
            for i in range(1, self._ef_num + 1):
                joint_name =  self._links[-i].getJoint().getName().encode("utf-8") 
                if joint_name in self._joint_limits:
                    joints[-i] = np.random.uniform(self._joint_limits[joint_name][0]*180/np.pi, \
                                    self._joint_limits[joint_name][1]*180/np.pi);  #0.04
        return joints          
    
    def check_joint_limits(self, joint_values, base_link='right_arm_mount'):
        """
        Check the joint limits based on urdf
        """
        num = self._get_base_idx_shifted(base_link)
        joint_values = deg2rad(joint_values)
        for idx in range(joint_values.shape[0]):  
            joint_name = self._joint_name[num + idx]
            if joint_name in self._joint_limits:
                lower_bound_check = joint_values[idx] >= self._joint_limits[joint_name][0]
                upper_bound_check = joint_values[idx] <= self._joint_limits[joint_name][1]
                if not (lower_bound_check and upper_bound_check):
                    print("{} joint limits exceeded! value: {}".format(joint_name, joint_values[idx]))
                    return False
        return True

    def gen_rand_pose(self, base_link='right_arm_mount', base_pose=None):
        """
        Generate random poses given base link or base pose
        """
        joint_values = []
        margin = 0.1 # 0.1 
        num = self._get_base_idx_shifted(base_link)
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
        """
        load the offset from the presaved txt file (fixed link and repeated finger offset are added)
        """ 
        cur_path = os.path.abspath(os.path.dirname(__file__))
        offset_file = os.path.join(cur_path, self._name + '_models', 'center_offset.txt')
        offset = np.loadtxt(offset_file).astype(np.float32)
        offset_list = []
        for i in range(offset.shape[0]):
            if not offset[i, :].any():
               self._empty_link_ids.append(i)  #mark the empty link id and skip
               continue
            offset_pose = np.eye(4)
            offset_pose[:3, 3] = offset[i, :]
            if i == 10 and self._name == 'panda_arm': 
                # right finger has origin flipped in urdf and kdl does not support :(
                offset_pose[:3, :3] = rotZ(np.pi)[:3, :3]  
                offset_pose[1, 3] -= 0.026262  
            offset_list.append(offset_pose)
        return offset_list
        
    def offset_pose_center(self, pose, dir='off', base_link='right_arm_mount'): 
        """
        Off means from original pose to the centered pose in model coordinate.
        """ 
        base_idx = self._get_base_idx_shifted(base_link)
        input_list = False  
        if type(pose) == list:
            input_list = True
            pose = list2M(pose)
        for i in range(pose.shape[-1]):
            offset_pose = self.center_offset[base_idx + i].copy()
            if dir == 'on':     
                offset_pose[:3, 3] *= -1 
            pose[:, :, i] = pose[:, :, i].dot(offset_pose)
        if input_list:
            return M2list(pose)
        return pose
    
    def get_link_name(self, name=None, link_id=None):
        if name:
            return name.strip().split('_')[-1]
        elif link_id:
            if link_id == -1:
                return self._base_link.split('_')[-1]
            return self._arm_chain.getSegment(link_id).getName()

    def _get_base_idx_shifted(self, base_link):
        return self._link_names.index(base_link)

    def inverse_kinematics(self, position, orientation=None, seed=None):
        pos = PyKDL.Vector(position[0], position[1], position[2])
        if orientation is not None:
            rot = PyKDL.Rotation()
            rot = rot.Quaternion(orientation[0], orientation[1],
                                 orientation[2], orientation[3])
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if seed is None:
            _, seed = self.gen_rand_pose(self._base_link)
            seed = seed[:self._num_jnts]
            seed = deg2rad(seed)
        seed_array.resize(len(seed))
        for idx in range(seed.shape[0]):
            seed_array[idx] = seed[idx]

        # Make IK Call
        if orientation is not None:
            goal_pose = PyKDL.Frame(rot, pos)
        else:
            goal_pose = PyKDL.Frame(pos)
        result_angles = PyKDL.JntArray(self._num_jnts)

        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles))
            return result
        else:
            return None

    def forward_position_kinematics(self, joint_values=None):
        end_frame = PyKDL.Frame()
        kdl_array = PyKDL.JntArray(self._num_jnts)
        for idx in range(joint_values.shape[0]):
            kdl_array[idx] = joint_values[idx]
        self._fk_p_kdl.JntToCart(kdl_array,
                                 end_frame)
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        return np.array([pos[0], pos[1], pos[2],
                         rot[0], rot[1], rot[2], rot[3]])

def main():    
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, default='panda_arm', help='Robot Name')
    parser.add_argument('--test', type=str, default='all', help='Robot Name')
    args = parser.parse_args()
    camera_intrinsics=np.array([[525, 0, 319.5],[ 0, 525, 239.5],[0, 0, 1]])    
    camera_extrinsics=np.array([[-0.211719, 0.97654, -0.0393032, 0.377451],[0.166697, -0.00354316, \
     -0.986002, 0.374476],[-0.96301, -0.215307, -0.162036, 1.87315],[0,0, 0, 1]])
    width = 640 
    height = 480
    camera_pos = np.array([0.6, -1.8, 1.2])
    
    mkdir_if_missing('test_image')
    renderer = YCBRenderer(width=width, height=height, render_marker=False, robot=args.robot)
    if args.robot == 'panda_arm':
        models = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'hand', 'finger', 'finger']
        base_link = 'panda_link0'
        if args.test == 'middle':
            base_link = 'panda_link3'
        if args.test == 'end':
            base_link = 'panda_link7'
        base_idx = 0
        name = base_link.strip().split('_')[-1]
        if name in models:
            base_idx = models.index(name) + 1 # take the link name
        obj_paths = [
            '{}_models/{}.DAE'.format(args.robot,item) for item in models]
        colors = [
            [0.1*(idx+1),0,0] for idx in range(len(models))]
        texture_paths = ['' for item in models]
        cls_indexes = range(base_idx, len(models))
        robot = robot_kinematics(args.robot)
    elif args.robot == 'baxter':
        models = ['S0', 'S1', 'E0', 'E1', 'W0', 'W1', 'W2', 'electric_gripper_base', 'extended_narrow', 'extended_narrow']
        base_idx = 0
        base_link = 'right_arm_mount'
        if args.test[:6] == 'middle':
            base_link = 'right_upper_elbow'
            base_idx = 3
        obj_paths = [
            '{}_models/{}.DAE'.format(args.robot,item) for item in models]
        colors = [
            [0.1*(idx+1),0,0] for idx in range(len(models))]
        texture_paths = ['' for item in models]
        cls_indexes = range(base_idx, len(models))
        robot = robot_kinematics(args.robot, 'right_arm_mount', 'right_gripper_base')

    renderer.load_objects(obj_paths, texture_paths, colors)
    renderer.set_camera_default()
    renderer.set_projection_matrix(640, 480, 525, 525, 319.5, 239.5, 0.001, 1000)
    renderer.set_light_pos([0, 0, 1])
    image_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
    seg_tensor = torch.cuda.FloatTensor(height, width, 4).detach()
    
    for index in range(10):
        arm_test_image = np.zeros([height, width, 3], dtype=np.uint8)
        renderer.set_camera(camera_pos, 2*camera_pos, [0,0,-1])
        renderer.set_light_pos(camera_pos + np.random.uniform(-0.5, 0.5, 3))
        intensity = np.random.uniform(0.8, 2)
        light_color = intensity * np.random.uniform(0.9, 1.1, 3)
        renderer.set_light_color(light_color)  
        rand_base = np.eye(4)
        quat = np.random.randn(4)
        rand_base[:3,:3] = quat2mat(quat / np.linalg.norm(quat))
        poses_p, joint = robot.gen_rand_pose(base_link, base_pose=rand_base)[:len(cls_indexes)]
        joint_test = robot.solve_poses_from_joint(np.array(joint), base_link, base_pose=np.eye(4))
        print('joint test ===================')
        print(joint)
        print(robot.solve_joint_from_poses(joint_test, base_link, base_pose=np.eye(4)))
        print('======================')

        print('mesh center test ===================')
        poses_p = robot.offset_pose_center(poses_p, dir='off', base_link=base_link) 
        poses_p = robot.offset_pose_center(poses_p, dir='on', base_link=base_link)  #center -> original pose
        print(deg2rad(robot.solve_joint_from_poses(poses_p, base_link, base_pose=np.eye(4))))

        poses_p = robot.offset_pose_center(poses_p, dir='off', base_link=base_link)  #original -> center pose
        poses_p = robot.offset_pose_center(poses_p, dir='on', base_link=base_link) 
        print(deg2rad(robot.solve_joint_from_poses(poses_p, base_link, base_pose=np.eye(4))))

        print('perturb test ===================')
        poses_p, perturb = robot.perturb_pose(poses_p, base_link, base_pose=rand_base)
        poses_p = robot.offset_pose_center(poses_p, dir='off', base_link=base_link)
        poses_p = robot.offset_pose_center(poses_p, dir='on', base_link=base_link)
        poses_p, perturb = robot.perturb_pose(poses_p, base_link, base_pose=rand_base, center_offset=True)
        print('======================')
        
        poses = []
        for i in range(len(poses_p)):
            pose_i = poses_p[i]
            rot = mat2quat(pose_i[:3,:3])
            trans = pose_i[:3,3]
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
        arm_test_image[mask[:,:,2]!=0] = image[mask[:,:,2]!=0] #red channel
        cv2.imwrite( 'test_image/%06d-color.png' % index, arm_test_image)
        if args.test == 'all': 
            print('IK test ======================')
            # ros quat xyzw | transforms3d wxyz
            _, joints = robot.gen_rand_pose(base_link)
            joints = deg2rad(joints[:7])
            p = robot.forward_position_kinematics(joint_values=np.array(joints))
            pos = p[:3]
            rot = p[3:]
            joints =  robot.inverse_kinematics(pos, rot)
            if joints is not None:
                p_ = robot.forward_position_kinematics(joint_values=joints) #
            else:
                print('no solution found by ik solver')
            pos_view = renderer.V[:3,:3].dot(pos) + renderer.V[:3, 3]
            center = camera_intrinsics.dot(pos_view)
            center = center / center[2]
            arm_test_image =  np.zeros([height, width, 3], dtype=np.uint8)
            x = int(center[0])
            y = int(center[1])
            
            if joints is not None:
                joints = rad2deg(joints)
                joints_ = np.zeros(joints.shape[0] + 3)
                joints_[:joints.shape[0]] = joints       
                p_ = robot.solve_poses_from_joint(joints_[:-2], base_link, base_pose=np.eye(4))[-1]
                print(p_[:3, 3], pos)
                poses_p = robot.solve_poses_from_joint(np.array(joints_), base_link, base_pose=np.eye(4))
                poses_p = robot.offset_pose_center(poses_p, dir='off', base_link=base_link)
                poses = []
                for i in range(len(poses_p)):
                    pose_i = poses_p[i]
                    rot = mat2quat(pose_i[:3,:3])
                    trans = pose_i[:3,3]
                    poses.append(np.hstack((trans,rot))) 
                renderer.set_poses(poses)
                renderer.render(range(joints_.shape[0]), image_tensor, seg_tensor)
                image_tensor = image_tensor.flip(0)
                seg_tensor = seg_tensor.flip(0)
                im = image_tensor.cpu().numpy()
                im = np.clip(im, 0, 1)
                im = im[:, :, (2, 1, 0)] * 255
                image = im.astype(np.uint8)

                mask = cv2.cvtColor(seg_tensor.cpu().numpy(), cv2.COLOR_RGB2BGR)
                arm_test_image[y-20:y+20, x-20:x+20, :] = np.array([255, 0, 0])
                arm_test_image[mask[:,:,2]!=0] = image[mask[:,:,2]!=0] 

                cv2.imwrite( 'test_image/%06d-color.png' % index, arm_test_image) #over write
            print('======================')


if __name__ == "__main__":
    main()
