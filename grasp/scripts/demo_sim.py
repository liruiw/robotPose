#!/usr/bin/env python
#
# Copyright (c) 2018, NVIDIA  All rights reserved.
#
# Finds Franka's end-effector 'right_gripper' frame relative to a named object
# in the scene and saves it as the named pickle file.
#
# The file, specified by --grasp_rospath, should contain a right_gripper pose
# defined as a geometry_msgs/TransformStamped whose header.frame_id specifies
# the frame the pose is defined in, and in particular the object that the pose
# is being used to manipulate. This file describes a manipulation behavior
# entirely described relative to that object.

from __future__ import print_function

from lula_pyutil import math_util, util
from lula_pyutil.math_util import (
        pack_transform, numpy_vec, numpy_quat, decompose_transform_matrix)
from lula_franka.franka import Franka
from lula_control.world import make_basic_world
from lula_control.object import ControllableObject
from lula_control.frame_commander import RobotConfigModulator
from lula_ros_util_internal.msg import RosVector
from lula_ros_util_internal.srv import (
    RosVectorService,
    RosVectorServiceRequest,
    RosVectorServiceResponse)

import os
import signal
import rospy
import tf
import sys
import argparse
import copy
import numpy as np
import numpy.linalg as la
import pickle
import threading
import roslib
roslib.load_manifest('joint_states_listener')
from joint_states_listener.srv import ReturnJointStates
roslib.load_manifest('grasps_listener')
from grasps_listener.srv import ReturnGrasps
from transforms3d.quaternions import mat2quat, quat2mat
from robotPose.robot_pykdl import *

def ros_quat(tf_quat): #wxyz -> xyzw
    quat = np.zeros(4)
    quat[-1] = tf_quat[0]
    quat[:-1] = tf_quat[1:]
    return quat

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

def call_return_grasps():
    rospy.wait_for_service("return_grasps")
    try:
        s = rospy.ServiceProxy("return_grasps", ReturnGrasps)
        resp = s()
    except rospy.ServiceException, e:
        print("error when calling return_grasps: %s"%e)
        sys.exit(1)

    return (resp.position, resp.orientation)

def grasp_publisher(br, RT):

    world_frame = '00_base_link'
    source_frame = 'selected_grasp'
    br.sendTransform(RT[:3, 3], ros_quat(mat2quat(RT[:3, :3])), rospy.Time.now(), source_frame, world_frame)


if __name__ == '__main__':
    name = 'grasp_object'
    parser = argparse.ArgumentParser(name)
    parser.add_argument(
            '--apply_to',
            type=str,
            default='00_sugar_box',
            help='If set, applies the grasp defined in the grasp data file specified by --grasp_rospath to the named object.')
    parser.add_argument(
            '--use_gripper',
            action='store_true',
            help='If set, uses the gripper, otherwise it doesn\'t.')
    parser.add_argument(
            '--use_grasp',
            type=int,
            default=-1,
            help='If set, choose the grasp number')
    parser.add_argument(
            '--real_robot',
            type=bool,
            default=True,
            help='If set, run on real robot')

    joint_names = ["panda_joint1",
                   "panda_joint2",
                   "panda_joint3",
                   "panda_joint4",
                   "panda_joint5",
                   "panda_joint6",
                   "panda_joint7"]

    args = parser.parse_args()
    obj_name = args.apply_to
    pause_between_steps = True
    robot = robot_kinematics('panda_arm')

    # ros node
    rospy.init_node(name)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    # read grasps from the grasp planner
    while 1:
        (positions, orientations) = call_return_grasps()
        num = len(positions) / 3
        if num > 0:
            break

    pose_grasp = np.zeros((num, 4, 4), dtype=np.float32)
    for i in range(num):
        pose_grasp[i, :3, 3] = positions[3*i:3*i+3]
        x = orientations[4 * i]
        y = orientations[4 * i + 1]
        z = orientations[4 * i + 2]
        w = orientations[4 * i + 3]
        pose_grasp[i, :3, :3] = quat2mat([w, x, y, z])
        pose_grasp[i, 3, 3] = 1.0

    print(pose_grasp.shape)
    print(pose_grasp)

    if pause_between_steps:
        raw_input('Press key to continue')

    # read grasps from file
    # pose_grasp = np.loadtxt('output/%s_grasp_pose.txt' % obj_name[3:], delimiter=',')
    # pose_grasp = pose_grasp[:, :-2].reshape([pose_grasp.shape[0], 4, 4])
    # pose_grasp[:, :3, 3] /= 1000 # scale the translation

    # define the robot
    print('<franka robot>')
    config_modulator = RobotConfigModulator()
    franka = Franka(is_physical_robot=args.real_robot)

    # retrieve the pose of object
    target_frame = '00_base_link'
    source_frame = obj_name
    trans, rot = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
    qt = np.zeros((4,), dtype=np.float32)
    qt[0] = rot[3]
    qt[1] = rot[0]
    qt[2] = rot[1]
    qt[3] = rot[2]
    obj_T = np.eye(4)
    obj_T[:3, :3] = quat2mat(qt)
    obj_T[:3, 3] = trans
    obj_T_inv = np.linalg.inv(obj_T)

    print('Translation:\n', trans)
    print('Quaternion:\n', rot)
    print('Frame matrix:\n', obj_T)
    print('Frame matrix inv:\n', obj_T_inv)

    # select the closest grasp
    # 1. choose two anchor points
    # anchor 1 in base frame
    anchor1_base = np.array([0, 0, trans[2]])
    # anchor 2 in base frame
    anchor2_base = np.array([trans[0], trans[1], trans[2] + 0.5])

    # convert the two anchors in object frame
    anchor1_obj = np.dot(obj_T_inv[:3, :3], anchor1_base) + obj_T_inv[:3, 3]
    anchor2_obj = np.dot(obj_T_inv[:3, :3], anchor2_base) + obj_T_inv[:3, 3]

    # normalization
    anchor1_obj = anchor1_obj / np.linalg.norm(anchor1_obj)
    anchor2_obj = anchor2_obj / np.linalg.norm(anchor2_obj)

    # read joints
    (position, velocity, effort) = call_return_joint_states(joint_names)
    print(position)

    if args.use_grasp >= 0:
        index_min = args.use_grasp
    else:
        # for each grasp
        index_min = -1
        distance_min = 10000
        for i in range(pose_grasp.shape[0]):

            # check feasibility
            grasp_candidate = np.dot(obj_T, pose_grasp[i, :, :])
            pos = grasp_candidate[:3, 3]
            rot = ros_quat(mat2quat(grasp_candidate[:3, :3]))
            joints =  robot.inverse_kinematics(pos, rot, np.array(position))
            print('grasp %d' % i)
            print(joints)
            if joints is None:
                continue

            grasp = pose_grasp[i, :3, 3]
            grasp = grasp / np.linalg.norm(grasp)
            # compute cosine distance to the two anchors
            distance = np.arccos(np.dot(grasp, anchor1_obj)) + np.arccos(np.dot(grasp, anchor2_obj))
            if distance < distance_min:
                index_min = i
                distance_min = distance

    # select the grasp
    grasp_obj = pose_grasp[index_min, :, :]
    print('Select grasp #', index_min)
    print(grasp_obj)

    # transform to right gripper
    target_frame = 'panda_hand'
    source_frame = 'right_gripper'
    trans, rot = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
    offset_pose = np.eye(4)
    offset_pose[:3, 3] = trans
    grasp_obj = grasp_obj.dot(offset_pose)

    # offset center
    grasp_list = robot.offset_pose_center([grasp_obj], dir='off', base_link='panda_link7')
    grasp_obj = grasp_list[0]

    # transform hand pose from object to base
    grasp_T = np.dot(obj_T, grasp_obj)
    grasp_frame = math_util.unpack_transform_to_frame(grasp_T)

    # publish the grasp for visualization
    t = threading.Thread(target=grasp_publisher, args=(br, grasp_T))
    t.start()

    if pause_between_steps:
        raw_input('Press key to continue')

    # define the standoff pose
    print(grasp_T)
    grasp_standoff = grasp_T.copy()
    grasp_standoff[2, 3] += 0.1
    print(grasp_standoff)
    standoff_frame = math_util.unpack_transform_to_frame(grasp_standoff)

    print('<world client>')
    world = make_basic_world()
    controllable_obj = ControllableObject(world.get_object(obj_name), robot=franka)

    print('Moving to standoff...')
    franka.set_speed(speed_level='slow')
    franka.end_effector.go_local(standoff_frame)

    if pause_between_steps:
        raw_input('Press key to move to target')

    # compute standoff pose
    print('<opening gripper>')
    franka.end_effector.gripper.open(speed = .1, wait=False, actuate_gripper=args.use_gripper)

    print('Suppressing and moving to target...')
    controllable_obj.suppress()

    franka.set_speed(speed_level='slow')
    franka.end_effector.go_local(grasp_frame)

    if pause_between_steps:
        raw_input('Press key to continue')
    #exit(0)

    controllable_frames = franka.end_effector.gripper.close(
            controllable_obj, speed = .1, actuate_gripper=args.use_gripper)
    print('controllable frames:\n', controllable_frames)
    rospy.sleep(2.)

    if pause_between_steps:
        raw_input('Press key to pick up object')

    print('Picking object up...')
    bottom_frame = controllable_frames['bottom']
    init_frame = bottom_frame.frame

    print('init_frame:\n', init_frame)
    target_frame = copy.deepcopy(init_frame)
    target_frame['orig'] += np.array([0., 0., .1])
    bottom_frame.go_local(target_frame)

    if pause_between_steps:
        raw_input('Press any key move to target')

    print('Place back on table...')
    bottom_frame.go_local(init_frame)
    
    if pause_between_steps:
        raw_input('Press any key to open gripper (and detach)')
    print('<opening gripper>')

    franka.end_effector.freeze()
    franka.end_effector.gripper.open(speed = .1, wait=True, actuate_gripper=args.use_gripper)

    if pause_between_steps:
        raw_input('Press any key to move to standoff')

    print('Moving back to standoff...')
    franka.end_effector.go_local(standoff_frame)
    controllable_obj.unsuppress()
    franka.set_speed(speed_level='med')

    franka.end_effector.go_local(
            orig = [-0.03661174699664116, -0.41725921630859375, 0.2315673828125])

    print('Exiting successfully.')
