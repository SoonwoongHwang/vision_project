#!/usr/bin/env python3
import rospy
import pyrealsense2 as rs
import cv2
import numpy as np

import sys
sys.path.append("/home/hsw/catkin_ws/src/picking_vision/src")

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from rs_pcl_stream import VisionConnection



def optimalFrame(img_frame, depth_intrin):
    k = np.array([[depth_intrin.fx, 0, depth_intrin.ppx], [0, depth_intrin.fy, depth_intrin.ppy], [0, 0, 1]])
    d = np.array([depth_intrin.coeffs[0], depth_intrin.coeffs[1], depth_intrin.coeffs[2], depth_intrin.coeffs[3], depth_intrin.coeffs[4]])
        

    # print(aruco_img.shape)
    h, w = img_frame.shape[:2]

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 1, (w, h))
            
    dst = cv2.undistort(img_frame, k, d, None, newcameramtx)
    x, y, w1, h1 = roi
    dst = dst[y:y+h1, x:x+w1]
    vision_frame = dst

    return vision_frame, w1, h1

def run():

    # Create Subscribers
    # pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    bridge = CvBridge()
    rsc = VisionConnection()

    aruco_image_pub = rospy.Publisher('/aruco_detection_result_img', Image, queue_size=10)
    aruco_pose_Info_pub = rospy.Publisher('/aruco_pose_Info', PoseStamped, queue_size=10)

    obj_image_pub = rospy.Publisher('/ort_detection_result_img', Image, queue_size=10)
    obj_pose_Info_pub = rospy.Publisher('/obj_pose_Info', PoseStamped, queue_size=10)
    # rate = rospy.Rate(30)

    rsc.rs_connection_set()

    try:
        while not rospy.is_shutdown():
            if not rsc.point_cloud:
                img_frame = rsc.vision_frame_set()
                vision_frame, w, h = optimalFrame(img_frame, rsc.depth_intrin)
            else:
                # plc_frame = rsc.vision_frame_set()
                print("Not Yet!!")

    finally:
        rsc.pipeline.stop()

if __name__ == '__main__':
    rospy.init_node('pcl_object', anonymous=True)
    run()
    rospy.spin()
