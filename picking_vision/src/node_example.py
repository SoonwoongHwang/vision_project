'''
Author:   Hyun-Jun Hyung (hjhyung@kimi.re.kr)
Date:     2021/9/24
Revision: 
Purpose: This code is ROS publisher and subscribe 
         test code for now.
'''

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import numpy as np
import tensorflow as tf

import cv2


# This function is callback function for joint_state subscribe.
def joint_state_callback(msg):
	position = msg.position
	name = msg.name
	print("name: ", name)
	print("position: ", position)

# This function is callback funtion for test topic.
def test_callback(data):
	print(data)


# This function is joint_state subscribe test code.
def listener():
	rospy.init_node('listener', anonymous=True)
	# rospy.Subscriber("joint_states", JointState, joint_state_callback)                                 					# joint_states
	
	image_raw_sub = rospy.Subscriber("/color/image_raw", Image, test_callback, queue_size=1)                                # /color/image_raw
	image_raw_comp_sub = rospy.Subscriber("/color/image_raw/compressed", CompressedImage, test_callback, queue_size=1)      # /color/image_raw/compressed

	depth_raw_sub = rospy.Subscriber("/depth/image_rect_raw", Image, test_callback, queue_size=1)                           # /depth/image_rect_raw
	depth_raw_comp_sub = rospy.Subscriber("/depth/image_rect_raw/compressed", CompressedImage, test_callback, queue_size=1) # /depth/image_rect_raw/compressed
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


# This function is joint_state publisher test code.
def talker():
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rospy.init_node('joint_state_publisher')
	rate = rospy.Rate(2) # 10hz
	IRB1200_JointState = JointState()
	IRB1200_JointState.header = Header()
	IRB1200_JointState.header.stamp = rospy.Time.now()
	IRB1200_JointState.name = ['abb_irb1200_5_90joint_1', 'abb_irb1200_5_90joint_2',
	                  		   'abb_irb1200_5_90joint_3', 'abb_irb1200_5_90joint_4',
					  		   'abb_irb1200_5_90joint_5', 'abb_irb1200_5_90joint_6']

	joint_angle1 = 0.0
	joint_angle2 = 0.0
	joint_angle3 = 0.0
	joint_angle4 = 0.0
	joint_angle5 = 0.0
	joint_angle6 = 0.0
		
	while not rospy.is_shutdown():
		IRB1200_JointState.header.stamp = rospy.Time.now()
		
		joint_angle2 = joint_angle2 + 0.1
		joint_angle3 = joint_angle3 + 0.1
		joint_angle1 = joint_angle1 + 0.1
		joint_angle4 = joint_angle4 + 0.1
		joint_angle5 = joint_angle5 + 0.1
		joint_angle6 = joint_angle6 + 0.1

		print("-----------------------------------")
		print("joint_angle1: ", joint_angle1, "joint_angle2: ", joint_angle2, "\n"
			  "joint_angle3: ", joint_angle3, "joint_angle4: ", joint_angle4, "\n"
			  "joint_angle5: ", joint_angle5, "joint_angle6: ", joint_angle6)

		IRB1200_JointState.position = [joint_angle1, joint_angle2, joint_angle3,
		                      joint_angle4, joint_angle5, joint_angle6]


		pub.publish(IRB1200_JointState)

		rate.sleep()


# Tensorflow code



if __name__ == '__main__':
	# listener()
	# talker()

	# rospy.init_node('Image_node', anonymous=True)
	
	# # color/image_raw/compressed subsribe
	# sub = rospy.Subscriber("/color/image_raw/compressed", CompressedImage, test_callback, queue_size=1)
	
	# rospy.spin()

	rospy.init_node('rostensorflow')

	cv_bridge = CvBridge()
	print(tf.__version__)

	mnist = tf.keras.datasets.mnist

	(x_train, y_train), (x_test, y_test) = mnist.load_data()
	x_train, x_test = x_train / 255.0, x_test / 255.0

	model = tf.keras.models.Sequential([
		tf.keras.layers.Flatten(input_shape=(28, 28)),
		tf.keras.layers.Dense(128, activation='relu'),
		tf.keras.layers.Dropout(0.2),
		tf.keras.layers.Dense(10, activation='softmax')
	])

	model.compile(optimizer='adam',
				  loss='sparse_categorical_crossentropy',
				  metrics=['accuracy'])

	
	model.fit(x_train, y_train, epochs=5)

	model.evaluate(x_test,  y_test, verbose=2)
