#!/usr/bin/env python3

from pathlib import Path
from turtle import width
import cv2 as cv
from math import atan2, cos, sin, sqrt, pi
import numpy as np
from PIL import Image as pImage
from torch import rad2deg

from dis import dis
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def rpy_to_quaternion(rotation):

    q = quaternion_from_euler(rotation[0], rotation[1], rotation[2])

    return q

def drawAxis(img, p_, q_, color, scale):
  p = list(p_)
  q = list(q_)
 
  ## [visualization1]
  angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
  hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
 
  # Here we lengthen the arrow by a factor of scale
  q[0] = p[0] - scale * hypotenuse * cos(angle)
  q[1] = p[1] - scale * hypotenuse * sin(angle)
  cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)
 
  # create the arrow hooks
  p[0] = q[0] + 9 * cos(angle + pi / 4)
  p[1] = q[1] + 9 * sin(angle + pi / 4)
  cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)
 
  p[0] = q[0] + 9 * cos(angle - pi / 4)
  p[1] = q[1] + 9 * sin(angle - pi / 4)
  cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)
  ## [visualization1]
 
def getOrientation(pts, img):
  ## [pca]
  # Construct a buffer used by the pca analysis
  sz = len(pts)
  data_pts = np.empty((sz, 2), dtype=np.float64)
  for i in range(data_pts.shape[0]):
    data_pts[i,0] = pts[i,0,0]
    data_pts[i,1] = pts[i,0,1]
 
  # Perform PCA analysis
  mean = np.empty((0))
  mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)
  # print(eigenvectors, eigenvalues)
 
  # Store the center of the object
  cntr = (int(mean[0,0]), int(mean[0,1]))
  # print(cntr)
  ## [pca]
 
  ## [visualization]

  angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians

  # Draw the principal components
  cv.circle(img, cntr, 3, (255, 0, 255), 2)
  p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
  p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
  # print(p1, p2)
  drawAxis(img, cntr, p1, (0, 0, 255), 1)
  # drawAxis(img, cntr, p2, (0, 255, 0), 1)
 
  # Label with the rotation angle
  label = "  Rotation Angle: " + str(-int(np.rad2deg(angle))) + " degrees"
  textbox = cv.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
  cv.putText(img, label, (cntr[0], cntr[1]), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv.LINE_AA)
 
  return angle, cntr


def oriented_obj_dection_publisher():

  # ros node setting
  bridge = CvBridge()

  rospy.init_node('oriented_object', anonymous=False)
  image_pub = rospy.Publisher('/ort_detection_result_img', Image, queue_size=10)
  oriented_obj_Info_pub = rospy.Publisher('/oriented_obj_Info', PoseStamped, queue_size=10)
  rate = rospy.Rate(30)

  # realsense vision connection
  pipeline = rs.pipeline()
  config = rs.config()
  # print(config)

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
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

	# Start streaming
  profile = pipeline.start(config)

  align_to = rs.stream.color  
  align = rs.align(align_to)  

  try:
    while not rospy.is_shutdown():
		# Wait for a coherent pair of frames: depth and color
      frames = pipeline.wait_for_frames()
      # print(frames)

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
      #   continue

      depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
      color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
      depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)

      # depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
      # color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
      # depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
	        
		  # Convert images to numpy arrays
      # depth_image = np.asanyarray(depth_frame.get_data())
      # color_image = np.asanyarray(color_frame.get_data())
      depth_image = np.asanyarray(aligned_depth_frame.get_data())
      color_image = np.asanyarray(color_frame.get_data())

		  # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
      depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
    
      depth_colormap_dim = depth_colormap.shape
      color_colormap_dim = color_image.shape
    
      vision_img = color_image

      h, w, sh = vision_img.shape

      mid_x, mid_y = w//2, h//2
      # print(h, w)
      # print(mid_x, mid_y)

      cv.arrowedLine(vision_img, (mid_x, mid_y), (mid_x + 50, mid_y), (0,0,255), 2)
      cv.arrowedLine(vision_img, (mid_x, mid_y), (mid_x, mid_y + 50), (0,255,0), 2)
      # vision_img = results.render()
      # vision_img = np.asarray(vision_img)

      # k = cv.waitKey(1) & 0xFF

      # Load the image
      # input_img = pImage.open("/home/hsw/catkin_ws/src/picking_vision/src/match_img.jpg")
      # img_resize = input_img.resize((1280, 800))
      # img_resize.save("/home/hsw/catkin_ws/src/picking_vision/src/match_img_re.jpg")
      # match_shape = cv.imread("/home/hsw/catkin_ws/src/picking_vision/src/match_img_re.jpg")
      # # img = cv.imread("input_img.jpg")
  
      # # Was the image there?
      # if img is None:
      #   print("Error: File not found")
      #   exit(0)
        
      # cv.imshow('Input Image', match_shape)
  
      # Convert image to grayscale
      gray = cv.cvtColor(vision_img, cv.COLOR_BGR2GRAY)
      # matchgray = cv.cvtColor(match_shape, cv.COLOR_BGR2GRAY)
      # imgBlur = cv.GaussianBlur(gray,(5,5),1)
      # imgCanny = cv.Canny(imgBlur,100,100)

      # Convert image to binary
      _, bw = cv.threshold(gray, 20, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
      # _, mbw = cv.threshold(matchgray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
      binary = cv.bitwise_not(bw)
      cv.imshow('Binary', bw)
      cv.imshow('Binary Reverse', binary)
      cv.waitKey(1)
      # Find all the contours in the thresholded image
      contours, hierarchy = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
      # mcontours, _ = cv.findContours(mbw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
      # print(contours)
      # print(hierarchy)
      # matches = []
      for i, c in enumerate(contours):

        # Calculate the area of each contour
        area = cv.contourArea(c)
  
        # Ignore contours that are too small or too large
        if area < 3700 or 100000 < area:
          continue

        # match = cv.matchShapes(mcontours[0], c, cv.CONTOURS_MATCH_I2, 0.0)
        # matches.append((match, c))
        # cv.putText(vision_img, '{:0.2f}'.format(match), tuple(c[0][0]), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
        # matches.sort(key=lambda x:x[0])
        # Draw each contour only for visualisation purposes
        # print(c)
        cv.putText(vision_img, '{:0.2f}'.format(area), tuple(c[0][0]), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
        cv.drawContours(vision_img, contours, i, (0, 0, 255), 2)
  
        # Find the orientation of each shape
        angleR, centerPose = getOrientation(c, vision_img)

        dist = aligned_depth_frame.get_distance(centerPose[0], centerPose[1])
        depth_pixel = [centerPose[0], centerPose[1]]
        # print(depth_intrin)
        # print(depth_pixel)
        # print(dist)
        # print(depth_scale)
        # print(dist*depth_scale)
        depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dist*depth_scale)
        x_dist = round(depth_point[0]*1000, 4)
        y_dist = round(depth_point[1]*1000, 4)
        z_dist = round(depth_point[2]*1000, 4)
        # print(x_dist, y_dist, z_dist)

        rotation = [0, 0, angleR]

        q_ = rpy_to_quaternion(rotation)
              # print("Quaternion: ", q_)

        oriented_obj_Info = PoseStamped()
        oriented_obj_Info.pose.position.x = x_dist
        oriented_obj_Info.pose.position.y = y_dist
        oriented_obj_Info.pose.position.z = z_dist
        oriented_obj_Info.pose.orientation.x = q_[0]
        oriented_obj_Info.pose.orientation.y = q_[1]
        oriented_obj_Info.pose.orientation.z = q_[2]
        oriented_obj_Info.pose.orientation.w = q_[3]

        oriented_obj_Info_pub.publish(oriented_obj_Info)

        # print(oriented_obj_Info)

      imgage_msg = bridge.cv2_to_imgmsg(vision_img, "bgr8")
      image_pub.publish(imgage_msg)

    #   cv.imshow('Output Image', vision_img)
    #   if k == 27:
    #     cv.waitKey(k)
    #     break

    # cv.destroyAllWindows()
  
    # Save the output image to the current directory
    # cv.imwrite("gun_output_img1.jpg", vision_img)
  finally:
    # Stop streaming
    pipeline.stop()
      
if __name__ == '__main__':
  oriented_obj_dection_publisher()