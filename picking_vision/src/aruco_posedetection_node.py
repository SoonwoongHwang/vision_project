#!/usr/bin/env python3

import numpy as np
import cv2
import sys
# from utils import ARUCO_DICT
import argparse
import time

from PIL import Image as pImage
from dis import dis
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def aruco_display(corners, ids, rejected, image):
	if len(corners) > 0:
		# flatten the ArUco IDs list
		ids = ids.flatten()
		# loop over the detected ArUCo corners
		for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned in
			# top-left, top-right, bottom-right, and bottom-left order)
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			# convert each of the (x, y)-coordinate pairs to integers
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			# compute and draw the center (x, y)-coordinates of the ArUco
			# marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			# draw the ArUco marker ID on the image
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			print("[Inference] ArUco marker ID: {}".format(markerID))
			# show the output image
	return image

def rpy_to_quaternion(rx, ry, rz):
   
    q = quaternion_from_euler(rx, ry, rz)

    return q

def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, mid_x, mid_y):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera
    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)

    rvec = np.zeros((1,1,3))
    tvec = np.zeros((1,1,3))

    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
            # print(rvec)
            # print(tvec)
            cv2.arrowedLine(frame, (mid_x, mid_y), (mid_x + 50, mid_y), (0,0,255), 2)
            cv2.arrowedLine(frame, (mid_x, mid_y), (mid_x, mid_y + 50), (0,255,0), 2)

    return frame, rvec, tvec, ids

def aruco_detection_publisher():

    # ap = argparse.ArgumentParser()
    # ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    # args = vars(ap.parse_args())

    # if ARUCO_DICT.get(args["type"], None) is None:
    #     print(f"ArUCo tag type '{args['type']}' is not supported")
    #     sys.exit(0)

    # ros node setting
    bridge = CvBridge()

    rospy.init_node('aruco_pose_detect', anonymous=False)
    image_pub = rospy.Publisher('/aruco_detection_result_img', Image, queue_size=10)
    aruco_pose_Info_pub = rospy.Publisher('/aruco_pose_Info', PoseStamped, queue_size=10)
    rate = rospy.Rate(30)

    # realsense vision connection
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False

    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
        
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)
        
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    
    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)

        # Start streaming
    profile = pipeline.start(config)

    align_to = rs.stream.color  
    align = rs.align(align_to)

    try:

        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()

            aligned_frames = align.process(frames)
    
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()

            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue
            
            # depth_frame = frames.get_depth_frame()
            # color_frame = frames.get_color_frame()

            # if not depth_frame or not color_frame:
            #     continue
            
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)
            # print(depth_intrin)
            # print(color_intrin)
                    
            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape
            
            aruco_img = color_image

            aruco_dict_type = cv2.aruco.DICT_ARUCO_ORIGINAL
            # aruco_dict_type = cv2.aruco.DICT_5X5_100
            # k = np.array([[638.297878*2.42, 0, 641.520437*1.30], [0, 638.68774*2.42, 365.744002*0.659], [0, 0, 1]])
            # d = np.array([-0.044848, 0.035995, -0.000144, -0.001623, 0])
            k = np.array([[depth_intrin.fx, 0, depth_intrin.ppx], [0, depth_intrin.fy, depth_intrin.ppy], [0, 0, 1]])
            d = np.array([depth_intrin.coeffs[0], depth_intrin.coeffs[1], depth_intrin.coeffs[2], depth_intrin.coeffs[3], depth_intrin.coeffs[4]])
            # print(k)
            # print(d)

            # print(aruco_img.shape)
            h, w, c = aruco_img.shape

            mid_x, mid_y = w//2, h//2
            # print(mid_x, mid_y)
            
            aruco_img, rotation, translation, ids = pose_esitmation(aruco_img, aruco_dict_type, k, d, mid_x, mid_y)
            # rpy = np.array(rotation)
            # print(rotation.shape)

            rx = rotation[0][0][0]
            ry = rotation[0][0][1]
            rz = rotation[0][0][2]
            
            q_ = rpy_to_quaternion(rx, ry, rz)
              # print("Quaternion: ", q_)

            aruco_pose_Info = PoseStamped()
            aruco_pose_Info.header = ids
            aruco_pose_Info.pose.position.x = translation[0][0][0]
            aruco_pose_Info.pose.position.y = translation[0][0][1]
            aruco_pose_Info.pose.position.z = translation[0][0][2]
            aruco_pose_Info.pose.orientation.x = q_[0]
            aruco_pose_Info.pose.orientation.y = q_[1]
            aruco_pose_Info.pose.orientation.z = q_[2]
            aruco_pose_Info.pose.orientation.w = q_[3]

            aruco_pose_Info_pub.publish(aruco_pose_Info)
            print(aruco_pose_Info)

            imgage_msg = bridge.cv2_to_imgmsg(aruco_img, "bgr8")
            image_pub.publish(imgage_msg)

    finally:
        pipeline.stop()

if __name__ == '__main__':
    aruco_detection_publisher()