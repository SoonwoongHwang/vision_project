#!/usr/bin/env python3

'''
Author:   Hyun-Jun Hyung (hjhyung@kimi.re.kr)
Date:     2021/11/18
Revision: 
Purpose: It is code that rocognize object and publish recognized result(image, name, position).
'''

from dis import dis
import rospy

from sensor_msgs.msg import Image
from picking_vision.msg import ObjInfo

import numpy as np

import cv2
from cv_bridge import CvBridge

import torch

import pyrealsense2 as rs



def recognized_obj_publisher():
	bridge = CvBridge()

	rospy.init_node('recognized_object', anonymous=False)
	image_pub = rospy.Publisher('/yolov5_result_img', Image, queue_size=10)
	recognized_obj_Info_pub = rospy.Publisher('/recognized_obj_Info', ObjInfo, queue_size=10)
	rate = rospy.Rate(30)

	model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/hsw/work/yolov5/runs/train/exp4/weights/best.pt')
	# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
	# model = torch.hub.load('path/to/yolov5', 'custom', path='/home/hsw/work/yolov5_obb/runs/train/exp/weights/best.pt', source='local')

	# Intel realsense connect
	# Configure depth and color streams
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

	config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

	if device_product_line == 'L500':
		config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
	else:
		config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

	# Start streaming
	profile = pipeline.start(config)

	align_to = rs.stream.color  
	align = rs.align(align_to) 
	
	try:
		# while True:
		while not rospy.is_shutdown():
			# Wait for a coherent pair of frames: depth and color
			frames = pipeline.wait_for_frames()

			aligned_frames = align.process(frames)
	        
			depth_sensor = profile.get_device().first_depth_sensor()
			depth_scale = depth_sensor.get_depth_scale()

			# depth_frame = frames.get_depth_frame()
			# color_frame = frames.get_color_frame()
			# if not depth_frame or not color_frame:
			# 	continue

			aligned_depth_frame = aligned_frames.get_depth_frame()
			color_frame = aligned_frames.get_color_frame()
			
			if not aligned_depth_frame or not color_frame:
				continue

			depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
			color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
			depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)
	        
			# Convert images to numpy arrays
			depth_image = np.asanyarray(aligned_depth_frame.get_data())
			color_image = np.asanyarray(color_frame.get_data())

			# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
			depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

			depth_colormap_dim = depth_colormap.shape
			color_colormap_dim = color_image.shape
	        
			results = model(color_image, size=640)
			img = results.render()
			img = np.asarray(img)

			if len(results.pandas().xyxy[0].index) == 0:
				pass
			else:
				# get pos in image
				xmin = results.xyxy[0][0][0]
				xmin = xmin.to('cpu')
				ymin = results.xyxy[0][0][1]
				ymin = ymin.to('cpu')
				xmax = results.xyxy[0][0][2]
				xmax = xmax.to('cpu')
				ymax = results.xyxy[0][0][3]
				ymax = ymax.to('cpu')
		        
	            # get center pos
				ave_x = (xmin+xmax)/2
				ave_x = ave_x.numpy()
				ave_y = (ymin+ymax)/2
				ave_y = ave_y.numpy()

	            # get x, y, z distance from ir sensor.
				dist = aligned_depth_frame.get_distance(ave_x, ave_y)
				depth_pixel = [ave_x, ave_y]
				depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dist*depth_scale)
				x_dist = round(depth_point[0]*1000, 4)
				y_dist = round(depth_point[1]*1000, 4)
				z_dist = round(depth_point[2]*1000, 4)

	        # # Show images
	        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
	        # cv2.imshow('RealSense', img[0])
	        # cv2.waitKey(1)

	        # get recognized object info
			recognized_object = results.pandas().xyxy[0]["name"]
	        
			if recognized_object.size >= 1:
				obj_name = recognized_object[0]
				recognized_obj_Info = ObjInfo()
				recognized_obj_Info.obj_name = obj_name
				recognized_obj_Info.x = x_dist
				recognized_obj_Info.y = y_dist
				recognized_obj_Info.z = z_dist
			else:
				obj_name = ''
				x_dist = 0
				y_dist = 0
				z_dist = 0

				recognized_obj_Info = ObjInfo()
				recognized_obj_Info.obj_name = obj_name
				recognized_obj_Info.x = x_dist
				recognized_obj_Info.y = y_dist
				recognized_obj_Info.z = z_dist

			imgage_msg = bridge.cv2_to_imgmsg(img[0], "bgr8")
			image_pub.publish(imgage_msg)

			recognized_obj_Info_pub.publish(recognized_obj_Info)
			print(recognized_obj_Info)
			# print(depth_intrin)

	finally:
		# Stop streaming
		pipeline.stop()


if __name__ == '__main__':
	recognized_obj_publisher()

