import tensorflow as tf
from tensorflow import keras

import numpy as np
import matplotlib.pyplot as plt

import cv2
import pyrealsense2 as rs

import threading
import time

# Print Tensorflow version
print("tensorflow version: ", tf.__version__)

# Check available GPU devices.
print("The following GPU devices are available: %s" % tf.test.gpu_device_name())

def load_data():
	fashion_mnist = keras.datasets.fashion_mnist
	(train_images, train_labels), (test_images, test_labels) = fashion_mnist.load_data()

	class_names = ['T-shirt/top', 'Trouser', 'Pullover', 'Dress', 'Coat',
               'Sandal', 'Shirt', 'Sneaker', 'Bag', 'Ankle boot']

	train_images = train_images / 255.0
	test_images = test_images / 255.0
	
	return (train_images, train_labels), (test_images, test_labels), class_names


def load_model(model_name):
	loaded_model = tf.keras.models.load_model(model_name)

	return loaded_model


def set_realsense():
	global color_image
	color_image = 0
	
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
	pipeline.start(config)

	try:
		while True:
			# Wait for a coherent pair of frames: depth and color
			frames = pipeline.wait_for_frames()
			depth_frame = frames.get_depth_frame()
			color_frame = frames.get_color_frame()
			if not depth_frame or not color_frame:
				continue

			# Convert images to numpy arrays
			depth_image = np.asanyarray(depth_frame.get_data())
			color_image = np.asanyarray(color_frame.get_data())
			
			# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
			depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

			depth_colormap_dim = depth_colormap.shape
			color_colormap_dim = color_image.shape
			
			# If depth and color resolutions are different, resize color image to match depth image for display
			if depth_colormap_dim != color_colormap_dim:
				resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
				images = np.hstack((resized_color_image, depth_colormap))
			else:
				images = np.hstack((color_image, depth_colormap))

			# Show images
			# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
			# cv2.imshow('RealSense', gray_image)
			# cv2.waitKey(1)

	finally:
		# Stop streaming
		pipeline.stop()


if __name__ == '__main__':
	# Add thread for camera
	camera_thread = threading.Thread(target=set_realsense)
	camera_thread.start()

	# load dataset and model
	(train_images, train_labels), (test_images, test_labels), class_names = load_data()
	loaded_model = load_model('my_model')

	# realtime classfication 
	while True:
		gray_image = tf.image.rgb_to_grayscale(color_image)
		gray_image_cv = np.uint8(gray_image)
		
		cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		cv2.imshow('RealSense', color_image)
		cv2.waitKey(1)

		# data resize for inputting data to tensorflow
		gray_image = tf.image.resize(gray_image, [28,28])
		gray_image = np.uint8(gray_image)
		gray_image = gray_image / 255.0
		
		if np.size(gray_image) != 1:
			gray_image = (np.expand_dims(gray_image, 0))
			predictions_single = loaded_model.predict(gray_image)
			print("predict result: ", class_names[np.argmax(predictions_single[0])])
