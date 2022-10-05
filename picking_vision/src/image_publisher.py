#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

cap = cv2.VideoCapture(4)
print(cap.isOpened()) 
bridge = CvBridge()

def testDevice(source):
   cap = cv2.VideoCapture(source) 
   if cap is None or not cap.isOpened():
       print('Warning: unable to open video source: ', source)

def talker():
	pub = rospy.Publisher('/test', Image, queue_size=1)
	rospy.init_node('image_test', anonymous=False)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		ret, frame = cap.read()
		if not ret:
			break

		msg = bridge.cv2_to_imgmsg(frame, "bgr8")
		pub.publish(msg)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

		if rospy.is_shutdown():
			cap.release()


if __name__ == '__main__':
	
	# for i in range(0, 10):
	# 	testDevice(i)

	try:
		talker()
	except rospy.ROSInterruptException:
		pass



