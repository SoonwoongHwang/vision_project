#!/usr/bin/env python3

'''
Author:   Hyun-Jun Hyung (hjhyung@kimi.re.kr)
Date:     2021/11/18
Revision: 
Purpose: It is code that control the robot HH020 model.
'''


import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from picking_vision.msg import ObjInfo


def get_joint_angle(start_pos, end_pos):
	# This function is DR.Hwang works.
	joint_angle1 = 0
	joint_angle2 = 1
	joint_angle3 = 2
	joint_angle4 = 3
	joint_angle5 = 4
	joint_angle6 = 5

	joint_angle = [joint_angle1, joint_angle2, joint_angle3,
				   joint_angle4, joint_angle5, joint_angle6]

	return joint_angle

def send_command():
	print("send command for control robot")	


def callback(data):
	jointState_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

	HH020_JointState = JointState()
	HH020_JointState.header = Header()
	HH020_JointState.header.stamp = rospy.Time.now()
	HH020_JointState.name = ['joint_1', 'joint_2',
	                  		 'joint_3', 'joint_4',
					  		 'joint_5', 'joint_6']

	HH020_JointState.header.stamp = rospy.Time.now()
		
	joint_angles = get_joint_angle(1, 1)

	HH020_JointState.position = joint_angles

	jointState_pub.publish(HH020_JointState)
	send_command()


def robot_control_publisher():
	rospy.init_node('robot_control', anonymous=True)
	ObjInfo_sub = rospy.Subscriber("/recognized_obj_Info", ObjInfo, callback, queue_size=1)
	
	rospy.spin()

if __name__ == '__main__':
	robot_control_publisher()






