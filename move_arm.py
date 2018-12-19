import rospy
import baxter_interface
import tf
#from lib.pair_matching.RT_transform import *
import moveit_commander 
import numpy as np
import sys
def deg2rad(joint):
	return np.array(joint)*(np.pi/180.0)
def rad2deg(joint):
	return np.array(joint)*(180./np.pi)
def main():
	import argparse
	parser = argparse.ArgumentParser()
	parser.add_argument('--robot', type=str, default='baxter', help='Robot Name')
	args = parser.parse_args()
	if args.robot == 'baxter':
		joint_state_topic = ['joint_states:=/robot/joint_states']
		moveit_commander.roscpp_initialize(joint_state_topic)
		rospy.init_node('move', anonymous=True)
		robot = moveit_commander.RobotCommander()
		gr = moveit_commander.MoveGroupCommander("right_arm")
		#sample_data/000001 joint angles
		set_joint = deg2rad([-11.44393148, -21.72985943,  64.88937589, 77.6455835,  5.26733262,71.97212335,  -2.96916723])
		gr.set_joint_value_target(set_joint)
		plan = gr.go(wait=True)
		print rad2deg(gr.get_current_joint_values())
	elif args.robot == 'panda_arm':
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move', anonymous=True)
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group_name = "panda_arm"
		#correspond with synthetic
		set_joint = deg2rad([-21.44609135,-56.99551849,-34.10630934,-144.2176713,-28.41103454,96.58738471,4.39702329])
		group = moveit_commander.MoveGroupCommander(group_name)
		group.set_joint_value_target(set_joint)
		plan = group.go(wait=True)
		print rad2deg(group.get_current_joint_values())
if __name__ == "__main__":
	main()
