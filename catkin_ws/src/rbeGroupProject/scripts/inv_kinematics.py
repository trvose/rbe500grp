#!/usr/bin/env python

import rospy
import numpy as np
import math
from rbeGroupProject.msg import calc_joints
from rbeGroupProject.msg import cartesian_pose
from transforms3d.euler import mat2euler, euler2mat

global num_links
num_links = 3

def callback(data):
	#rospy.loginfo(rospy.get_caller_id() + 'The joint angles are: %s', data.data)
	pose = np.array(data.pose)
	position = pose[:3]
	euler_angles = pose[3:]
	
	
	print 'The position of end-effector is: '
	print(position)
	print(euler_angles)
	transform = get_transformation(position,euler_angles)
	print('=============================================')

	q1,q2,q3 = get_inv_kinematics()
	publish_calc_joints(q1,q2,q3)

def publish_calc_joints(q1,q2,q3):
	pub = rospy.Publisher('calc_joint_params', calc_joints)
    
	msg = calc_joints()
	msg.q1 = q1	
	msg.q2 = q2
	msg.q3 = q3

	rospy.loginfo(msg)
	pub.publish(msg)	
	
def get_transformation(position,euler_angles):
	rot_mat = np.array(euler2mat(euler_angles[0],euler_angles[1],euler_angles[2]))
	transform = np.array([[rot_mat[0,0],rot_mat[0,1],rot_mat[0,2],position[0]],
						  [rot_mat[1,0],rot_mat[1,1],rot_mat[1,2],position[1]],
						  [rot_mat[2,0],rot_mat[2,1],rot_mat[2,2],position[2]],[0,0,0,1]])
	print(transform)
	return transform

def INV_kinematics():

    rospy.init_node('INV_kinematics', anonymous=True)
    rospy.Subscriber('get_pose', cartesian_pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    INV_kinematics()

    
