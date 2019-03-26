import rospy
import tf
import numpy as np
import argparse
import torch
from transforms3d.quaternions import mat2quat, quat2mat

import roslib
roslib.load_manifest('joint_states_listener')
from joint_states_listener.srv import ReturnJointStates
from robotPose.robot_pykdl import *
from se3 import *


ycb_classes = ('002_master_chef_can', '003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', \
               '007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana', '019_pitcher_base', \
               '021_bleach_cleanser', '024_bowl', '025_mug', '035_power_drill', '036_wood_block', '037_scissors', '040_large_marker', \
               '051_large_clamp', '052_extra_large_clamp', '061_foam_brick')

joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

def tf_quat(ros_quat): #xyzw -> wxyz
    quat = np.zeros(4)
    quat[0] = ros_quat[-1]
    quat[1:] = ros_quat[:-1]
    return quat

def ros_quat(tf_quat): #wxyz -> xyzw
    quat = np.zeros(4)
    quat[0] = tf_quat[1]
    quat[1] = tf_quat[2]
    quat[2] = tf_quat[3]
    quat[3] = tf_quat[0]
    return quat

def get_bb3D(extent):
    bb = np.zeros((3, 8), dtype=np.float32)
    
    xHalf = extent[0] * 0.5
    yHalf = extent[1] * 0.5
    zHalf = extent[2] * 0.5
    
    bb[:, 0] = [xHalf, yHalf, zHalf]
    bb[:, 1] = [-xHalf, yHalf, zHalf]
    bb[:, 2] = [xHalf, -yHalf, zHalf]
    bb[:, 3] = [-xHalf, -yHalf, zHalf]
    bb[:, 4] = [xHalf, yHalf, -zHalf]
    bb[:, 5] = [-xHalf, yHalf, -zHalf]
    bb[:, 6] = [xHalf, -yHalf, -zHalf]
    bb[:, 7] = [-xHalf, -yHalf, -zHalf]
    
    return bb


def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print("error when calling return_joint_states: %s"%e)
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print("joint %s not found!"%joint_name)
    return (resp.position, resp.velocity, resp.effort)


if __name__ == '__main__':

    name = 'compute_world_poses'
    parser = argparse.ArgumentParser(name)
    parser.add_argument('--camera', dest='camera', help='ros camera to use', default='kinect1', type=str)
    parser.add_argument('--debug', dest='debug', help='debug or not', default=False, type=bool)
    parser.add_argument('--robot', type=str, default='panda_arm', help='Robot Name')

    args = parser.parse_args()
    ros_camera = args.camera
    robot = robot_kinematics('panda_arm')

    rospy.init_node('deepim_tf_listener')
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    prefix = '00_'
    suffix = '_world'
    base_link = 'panda_link0'
    target_frame = ros_camera + '_depth_optical_frame'
    world_frame = '00_base_link'

    # load extents
    extents = np.loadtxt('extents.txt')

    rate = rospy.Rate(10.0)
    world_initialized = False
    while not rospy.is_shutdown():

        # compute RT_base
        if world_initialized == False:

            count = 0
            num = 10
            poses = np.zeros((num, 7), dtype=np.float32)
            while count < num:

                # check robot joints first
                (position, velocity, effort) = call_return_joint_states(joint_names)
                position = position + (0,)
                print('panda joints')
                print(position)

                # poses from joints
                joints = rad2deg(np.array(position))
                robot_poses = robot.solve_poses_from_joint(np.array(joints), base_link, base_pose=np.eye(4))
                robot_poses = robot.offset_pose_center(robot_poses, dir='off', base_link=base_link)
                pose_hand_base = robot_poses[-1]
                print(pose_hand_base)

                # look for hand pose in camera frame
                source_frame = 'deepim/01_hand'
                try:
                    (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
                    pose_hand = np.zeros((7,), dtype=np.float32)
                    pose_hand[:4] = tf_quat(rot)
                    pose_hand[4] = trans[0]
                    pose_hand[5] = trans[1]
                    pose_hand[6] = trans[2]
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                print('hand pose')
                print(pose_hand)

                # camera to base transform
                RT_hand = np.eye(4, dtype=np.float32)
                RT_hand[:3, :3] = quat2mat(pose_hand[:4])
                RT_hand[:3, 3] = pose_hand[4:]
                RT_base = np.dot(pose_hand_base, np.linalg.inv(RT_hand))
                print(RT_base)

                qt = mat2quat(RT_base[:3, :3])
                if qt[0] < 0:
                    qt *= -1
                poses[count, :4] = qt
                poses[count, 4:] = RT_base[:3, 3]
                count += 1
                print(count)

            # average the poses
            RT_base = np.eye(4, dtype=np.float32)
            RT_base[:3, :3] = quat2mat(averageQuaternions(poses[:, :4]))
            RT_base[:3, 3] = np.mean(poses[:, 4:], axis=0)

        # look for object poses in camera frame
        cls_indexes = []
        poses = []
        for i in range(len(ycb_classes)):
            if ycb_classes[i][3] == '_':
                source_frame = 'deepim/' + prefix + ycb_classes[i][4:]
            else:
                source_frame = 'deepim/' + prefix + ycb_classes[i] 
            try:
                (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
                cls_indexes.append(i)
                pose = np.zeros((7,), dtype=np.float32)
                pose[:4] = tf_quat(rot)
                pose[4] = trans[0]
                pose[5] = trans[1]
                pose[6] = trans[2]
                poses.append(pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        if len(cls_indexes) == 0:
            continue

        # print pose info
        for i in range(len(cls_indexes)):
            print(ycb_classes[cls_indexes[i]])
            print(poses[i])

        # look for hand pose in camera frame
        source_frame = 'deepim/01_hand'
        try:
            (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            pose_hand = np.zeros((7,), dtype=np.float32)
            pose_hand[:4] = tf_quat(rot)
            pose_hand[4] = trans[0]
            pose_hand[5] = trans[1]
            pose_hand[6] = trans[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pose_hand = None

        if world_initialized == False:
            # compute table pose
            points = np.zeros((3, len(cls_indexes) * 8), dtype=np.float32)
            for i in range(len(cls_indexes)):
                bb3d = get_bb3D(extents[cls_indexes[i]])
                # transform the 3D box
                RT = np.eye(4, dtype=np.float32)
                RT[:3, :3] = quat2mat(poses[i][:4])
                RT[:3, 3] = poses[i][4:]
                RT = np.dot(RT_base, RT)
                print(RT)

                x3d = np.ones((4, 8), dtype=np.float32)
                x3d[0:3, :] = bb3d
                x3d = np.matmul(RT, x3d)
                points[:, i*8:i*8+8] = x3d[:3, :]

            pose_table = np.zeros((7,), dtype=np.float32)
            pose_table[0] = 1
            pose_table[1] = 0
            pose_table[2] = 0
            pose_table[3] = 0
            if len(cls_indexes) > 0:
                pose_table[4] = np.mean(points[0, :]) - 0.9
                pose_table[5] = np.mean(points[1, :]) - 0.5
                pose_table[6] = points[2, :].min() - 0.8
                print(pose_table)

            pose_table_obstacle = pose_table.copy()
            pose_table_obstacle[4] += 0.9
            pose_table_obstacle[5] += 0.5
            pose_table_obstacle[6] += 0.3

            # wall left
            pose_wall_left = np.zeros((7,), dtype=np.float32)
            pose_wall_left[0] = 1
            pose_wall_left[1] = 0
            pose_wall_left[2] = 0
            pose_wall_left[3] = 0
            if len(cls_indexes) > 0:
                pose_wall_left[4] = np.mean(points[0, :])
                pose_wall_left[5] = np.min(points[1, :]) - 0.5
                pose_wall_left[6] = np.mean(points[2, :])

            # wall right
            pose_wall_right = np.zeros((7,), dtype=np.float32)
            pose_wall_right[0] = 1
            pose_wall_right[1] = 0
            pose_wall_right[2] = 0
            pose_wall_right[3] = 0
            if len(cls_indexes) > 0:
                pose_wall_right[4] = np.mean(points[0, :])
                pose_wall_right[5] = np.max(points[1, :]) + 0.5
                pose_wall_right[6] = np.mean(points[2, :])

            # wall top
            pose_wall_top = np.zeros((7,), dtype=np.float32)
            pose_wall_top[0] = 1
            pose_wall_top[1] = 0
            pose_wall_top[2] = 0
            pose_wall_top[3] = 0
            if len(cls_indexes) > 0:
                pose_wall_top[4] = np.mean(points[0, :])
                pose_wall_top[5] = np.mean(points[1, :])
                pose_wall_top[6] = np.max(points[2, :]) + 0.5

            world_initialized = True

        # publish poses
        for i in range(len(cls_indexes)):
            # object pose
            RT = np.eye(4, dtype=np.float32)
            RT[:3, :3] = quat2mat(poses[i][:4])
            RT[:3, 3] = poses[i][4:]
            RT = np.dot(RT_base, RT)

            if ycb_classes[i][3] == '_':
                source_frame = prefix + ycb_classes[cls_indexes[i]][4:]
            else:
                source_frame = prefix + ycb_classes[cls_indexes[i]]
            br.sendTransform(RT[:3, 3], ros_quat(mat2quat(RT[:3, :3])), rospy.Time.now(), source_frame, world_frame)

        # hand pose
        if pose_hand is not None:
            RT = np.eye(4, dtype=np.float32)
            RT[:3, :3] = quat2mat(pose_hand[:4])
            RT[:3, 3] = pose_hand[4:]
            RT = np.dot(RT_base, RT)
            source_frame = 'panda_hand' + suffix
            br.sendTransform(RT[:3, 3], ros_quat(mat2quat(RT[:3, :3])), rospy.Time.now(), source_frame, world_frame)

        # table pose
        source_frame = 'table' + suffix
        br.sendTransform(pose_table[4:7], ros_quat(pose_table[:4]), rospy.Time.now(), source_frame, world_frame)

        source_frame = 'table_obstacle'
        br.sendTransform(pose_table_obstacle[4:7], ros_quat(pose_table_obstacle[:4]), rospy.Time.now(), source_frame, world_frame)

        source_frame = 'wall_left'
        br.sendTransform(pose_wall_left[4:7], ros_quat(pose_wall_left[:4]), rospy.Time.now(), source_frame, world_frame)

        source_frame = 'wall_right'
        br.sendTransform(pose_wall_right[4:7], ros_quat(pose_wall_right[:4]), rospy.Time.now(), source_frame, world_frame)

        source_frame = 'wall_top'
        br.sendTransform(pose_wall_top[4:7], ros_quat(pose_wall_top[:4]), rospy.Time.now(), source_frame, world_frame)

        # base in camera
        RT = np.linalg.inv(RT_base)
        br.sendTransform(RT[:3, 3], ros_quat(mat2quat(RT[:3, :3])), rospy.Time.now(), world_frame, target_frame)

        rate.sleep()
