#!/usr/bin/env python

import rospy
import sys

from rbeGroupProject.msg import joints
from rbeGroupProject.msg import calc_joints

calc_q1,calc_q2,calc_q3 = 0,0,0

def get_joints(data):
    global calc_q1,calc_q2,calc_q3

    calc_q1 = data.q1
    calc_q2 = data.q2
    calc_q3 = data.q3
    #print(data.q1,data.q2,data.q3)


    #return calc_q1,calc_q2,calc_q3

def params_publish(q1,q2,q3):
    
    pub = rospy.Publisher('joint_params', joints, queue_size=1)
    rospy.init_node('params_publish', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    msg = joints()
    msg.q1 = q1
    msg.q2 = q2
    msg.q3 = q3

    while not rospy.is_shutdown():

        rospy.loginfo(msg)
        pub.publish(msg)

        rospy.Subscriber('calc_joint_params', calc_joints, get_joints)

        print("For joint 1, obtained parameter = ",q1," and calculated parameter = ",calc_q1)
        print("Hence calculations done are ",(q1==calc_q1))

        print("For joint 2, obtained parameter = ",q2," and calculated parameter = ",calc_q2)
        print("Hence calculations done are ",(q2==calc_q2))

        print("For joint 3, obtained parameter = ",q3," and calculated parameter = ",calc_q3)
        print("Hence calculations done are ",(q3==calc_q3))

        rate.sleep()

    #rospy.spin()


    

if __name__ == '__main__':
    try:
        try:
          q1,q2,q3 = input('Enter the three joint variable values in degrees: ')
          params_publish(q1,q2,q3)

        except TypeError:
          print('Enter parameters in the format: q1,q2,q3')
          
           
    except rospy.ROSInterruptException:
        pass