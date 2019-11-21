#!/usr/bin/env python

import rospy
import numpy as np
import math
from rbeGroupProject.msg import joints
from rbeGroupProject.msg import cartesian_pose
from transforms3d.euler import mat2euler, euler2mat

global num_links
num_links = 3

def callback(data):
	#rospy.loginfo(rospy.get_caller_id() + 'The joint angles are: %s', data.data)
	q1 = data.q1
	q2 = data.q2
	q3 = data.q3

	# define length of links as required
	l1 = 0.2
	l2 = 0.2
	l3 = 0.2
	l4 = 0.1

	dh_param = calc_dh_param(q1,q2,q3,l1,l2,l3,l4)
	transform = calc_transform(dh_param)

	position = get_position(transform)
	euler_angles = get_euler_angles(transform)


	pub = rospy.Publisher('get_pose', cartesian_pose)
 	
 	msg = cartesian_pose()
	msg.pose = [position[0],position[1],position[2],euler_angles[0],euler_angles[1],euler_angles[2]]
	pub.publish(msg)	
	rospy.loginfo(msg)

	print 'The position of end-effector is: '
	print(position)
	print(euler_angles)
	print('=============================================')
	
def calc_dh_param(q1,q2,q3,l1,l2,l3,l4):
	theta = np.array([q1,q2,0]) 
	d = np.array([l1,0,l4+q3])
	a = np.array([l2,l3,0])
	alpha = np.array([0,90,0])

	params = np.array([theta,d,a,alpha])
	print 'The calculated DH parameters are: '
	print params
	print('-------------------------------------')
	return params

def calc_frame_transform(theta_i,d_i,a_i,alpha_i):
	c_theta = math.cos(math.radians(theta_i))
	s_theta = math.sin(math.radians(theta_i))

	c_alpha = math.cos(math.radians(alpha_i))
	s_alpha = math.sin(math.radians(alpha_i))

	A_i = np.array([[c_theta, -s_theta*c_alpha, s_theta*s_alpha,  a_i*c_theta],
				   [s_theta, c_theta*c_alpha,  -c_theta*s_alpha, a_i*s_theta],
				   [0,       s_alpha,           c_alpha,         d_i],
				   [0,       0,                 0,               1]])
	return A_i

def get_position(T):
	position = T[:3,3]
	return position

def get_euler_angles(T):
	euler  = mat2euler(T)
	return euler

def calc_transform(dh_param):
	T = np.identity(4)
	for i in range(len(dh_param.T)):
		theta_i = dh_param[0,i]
		d_i = dh_param[1,i]
		a_i = dh_param[2,i]
		alpha_i = dh_param[3,i]

		A_i = calc_frame_transform(theta_i,d_i,a_i,alpha_i)
		print 'Frame transform from frame ',i,' to frame ',i+1,' is: '
		print A_i
		print('-------------------------------------')

		T = np.matmul(T,A_i)

	print 'Final homogeneous tranformation matrix from frame 0 to frame ', num_links, ' is: '
	print T
	print('-------------------------------------')
	
	return T

def FW_kinematics():

    rospy.init_node('FW_kinematics', anonymous=True)
    rospy.Subscriber('joint_params', joints, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    FW_kinematics()

    
