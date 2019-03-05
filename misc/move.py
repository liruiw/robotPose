#!/usr/bin/env python
import os 
import sys
import copy
import rospy
import moveit_commander
import argparse
import moveit_msgs.msg
from shape_msgs.msg import SolidPrimitive, Mesh
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Vector3, PoseStamped
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_commander.conversions import pose_to_list, list_to_pose
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, BoundingVolume

def read_text_lines(file_path):
    f = open(file_path, 'r')
    lines = f.readlines()
    f.close()
    lines = [l.rstrip() for l in lines]
    return lines

def check_valid_range(left_goal, right_goal):
    #tuned threshold
    if left_goal.position.x > 0.30 and right_goal.position.x > 0.30 and abs(right_goal.position.y) < 0.4 \
        and abs(left_goal.position.y) < 0.4 and right_goal.position.z < 0.40 and left_goal.position.z < 0.40 \
        and left_goal.position.z > -0.1 and right_goal.position.z > -0.1:    
        return True
    else: return False
    #

def build_scene():#left arm is so close to the wall. Full constraint succeed:17. Without: 58.
    left_wall_pose = PoseStamped()
    left_wall_pose.header.frame_id = robot.get_planning_frame()
    left_wall_pose.pose.orientation.w = 1.0
    left_wall_pose.pose.position.y = 1
    left_wall_name = "left_wall"
    back_wall_pose = PoseStamped()
    back_wall_pose.header.frame_id = robot.get_planning_frame()
    back_wall_pose.pose.orientation.w = 1.0
    back_wall_pose.pose.position.x = -0.7
    back_wall_name = "back_wall"
    wall = [(left_wall_pose,left_wall_name), (back_wall_pose,back_wall_name)]
    return wall


def add_constraint():#left arm is so close to the wall. Full constraint succeed:17. Without: 58.
    rs0_left = JointConstraint();
    re0_left = JointConstraint();
    rs0_left.joint_name = "left_s0";  
    rs0_left.position = 0.0;
    rs0_left.tolerance_above = 1.6;
    rs0_left.tolerance_below = 0.0;
    rs0_left.weight = 0.8; 
    re0_left.joint_name = "left_e0";  
    re0_left.position = 0.0;
    re0_left.tolerance_above = 3;
    re0_left.tolerance_below = 0.8;
    re0_left.weight = 0.5;     
    constraint = Constraints()
    # constraint.joint_constraints.append(rs0_left)
    # constraint.joint_constraints.append(re0_left)
    return constraint

def check_boxes():
    start = rospy.get_time()
    seconds = rospy.get_time()
    timeout = 5
    while (seconds - start < timeout) and not rospy.is_shutdown():
        print scene.get_known_object_names()
        is_known = "left_wall" in scene.get_known_object_names() and "back_wall" in scene.get_known_object_names() 
  # Test if we are in the expected state
        if (is_known):
            return True
        rospy.sleep(0.1)
        seconds = rospy.get_time()
    return False


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--left_pose", type=str, required=False, help="input pose")
    parser.add_argument("--right_pose", type=str, required=False, help="input pose")
    parser.add_argument("--test_file_list", type=str, required=False, 
        default= '/home/liruiw/Projects/Optical-Manipulation/leroy_ws/src/move_arm/scripts/1000_target_poses.txt', 
    help="Path to the list of test files")
    args = parser.parse_args()
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('move', anonymous=True)
    publisher = rospy.Publisher('path_topic', String, queue_size=10)
    group_num = 90;
     # Instantiate a RobotCommander object.  This object is
    # an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    #add the wall as a box in the planning scene
    scene = moveit_commander.PlanningSceneInterface()
    wall = build_scene()
    rospy.sleep(2)
    scene.add_box(wall[0][1], wall[0][0], size=(4, 0.2, 4))
    scene.add_box(wall[1][1], wall[1][0], size=(0.2, 4, 4))
    if not check_boxes():
        moveit_commander.roscpp_shutdown()
    # planning_scene.world.collision_objects.push_back(remove_object);
    # planning_scene.world.collision_objects.push_back(remove_object);
    group = moveit_commander.MoveGroupCommander("both_arms")
    gr = moveit_commander.MoveGroupCommander("right_arm")
    gl = moveit_commander.MoveGroupCommander("left_arm")    
    # Planning to a Pose goal
    left_current_pose = gr.get_current_pose(end_effector_link='left_gripper').pose
    left_current_joint = gr.get_current_joint_values()#joint only has angles hard to tune
    right_current_pose = gl.get_current_pose(end_effector_link='right_gripper').pose
    group.set_max_velocity_scaling_factor(0.5);group.set_goal_tolerance(0.1);
    # group.set_planning_time(5);
    constraint = add_constraint()
    group.set_path_constraints(constraint)
    print group.get_path_constraints()
    # print group.get_active_joints()
    # plan to a random locations
    rospy.sleep(1)
    target_pose = []
    ite = 0; 
     #to test if the constraint is working
    # while(ite < group_num): #generate fixed number of moves
    #     left_goal = gl.get_random_pose(end_effector_link='left_gripper').pose
    #     right_goal = gr.get_random_pose(end_effector_link='right_gripper').pose
    #     if check_valid_range(left_goal, right_goal):
    #         target_pose.append((left_goal, right_goal))
    #         ite = ite + 1
    #         if ite % 10 == 0:
    #             print "ten valid targets just made"
    

    ################ fixed precalculated poses
    test_files = read_text_lines(args.test_file_list)
    for i in range(group_num):
        s = test_files[i]
        #print s,len(test_files)
        left_str= s[s.index("[") + 1:s.index("]")]
        right_str =s[s.rindex("[") + 1 :s.rindex("]")]
        left = map(float,left_str.split(","))
        right = map(float,right_str.split(","))
        #print left_str.split(",")
        left_goal = list_to_pose(left)
        right_goal = list_to_pose(right)
        #print left_goal
        target_pose.append((left_goal, right_goal))
 
    ################ for testing the constraints 
    
    #count succeed times
    count = 0
    with open('/home/liruiw/Projects/Optical-Manipulation/leroy_ws/src/move_arm/scripts/target_poses.txt', 'wr') as f:
        for i in range(group_num):
            path_num = "path%03d" % i
            print path_num
            #left = list_to_pose(args.left_pose.split (' ')); #right = list_to_pose(args.right_pose.split (' '))
            group.set_pose_target(target_pose[i][0], end_effector_link='left_gripper')
            group.set_pose_target(target_pose[i][1], end_effector_link='right_gripper')
            #keeps record of the data
            left_pose = pose_to_list(target_pose[i][0])
            right_pose = pose_to_list(target_pose[i][1])

            f.write("Pose Number(Left Right) %d,%s,%s\n" % (i, str(left_pose), str(right_pose)))
            publisher.publish(path_num)
            plan = group.go(wait=True) #plan should have enough time for receiving the message
            print plan
            # print "Joint Values", group.get_current_joint_values()
            if(plan): count=count+1;
            group.stop() #clear residue movement
            group.clear_pose_targets()
    publisher.publish("end_moving")
    print "Succeed Times:", count
    scene.remove_world_object("left_wall");    scene.remove_world_object("back_wall")
    moveit_commander.roscpp_shutdown()


#comments: Adding constraint can also be a dual-arm planning problem. Dual-arm group doesn't have 
#end_effector but two arms would have the synchronization problem.
    # bound = BoundingVolume()
    # primitives = shape_msgs.SolidPrimitive()
    # primitives.type = 1
    # primitives.dimensions = [0.3,0.4,0.5] #X, Y, Z
    # primitive_poses = Pose()
    # primitive_poses.position.x=0;primitive_poses.position.y=0;primitive_poses.position.z=0.15;

    # position_constraint = PositionConstraint()

    # bound.primitives = list((primitives))
    # bound.primitives_poses = list((primitives_poses))
    # # position_constraint.constraint_region = bound
    # position_constraint.weight = 0.8
    # print group.has_end_effector_link()
    # position_constraint.link_name = group.get_end_effector_link()
    # position_constraint.header.frame_id = group.get_planning_frame()
    # constraint = Constraints()
    # constraint.position_constraints.append(position_constraint)
    # group.set_path_constraints(constraint)
    

    # # mesh_pose = Pose()
    # offset = Vector3()
    # position_constraint.target_point_offset = offset