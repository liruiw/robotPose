import rospy
import baxter_interface
import tf
#from lib.pair_matching.RT_transform import *
import moveit_commander 
import numpy as np
def deg2ang(joint):
	return np.array(joint)*(np.pi/180.0)

limb.move_to_joint_positions(angles)
def main():
	import argparse
	parser = argparse.ArgumentParser()
	parser.add_argument('--robot', type=str, default='baxter', help='Robot Name')
	args = parser.parse_args()
	if args.robot == 'baxter':
		rospy.init_node('Hello_Baxter')
		limb = baxter_interface.Limb('right')
		set_joint = deg2ang([20,30,40,50,60,20,40]) #random angle?
		angles['right_s0']=set_joint[0]
		angles['right_s1']=set_joint[1]
		angles['right_e0']=set_joint[2]
		angles['right_e1']=set_joint[3]
		angles['right_w0']=set_joint[4]
		angles['right_w1']=set_joint[5]
		angles['right_w2']=set_joint[6]
if __name__ == "__main__":
	main()
