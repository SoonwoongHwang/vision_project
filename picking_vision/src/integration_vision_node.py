#!/usr/bin/env python3

from typing_extensions import Self
from matplotlib.pyplot import contour
import numpy as np
import cv2
import sys
sys.path.append("/home/hsw/catkin_ws/src/picking_vision/src")
# from utils import ARUCO_DICT
import argparse
import time
from rs_stream import VisionConnection

from PIL import Image as pImage
from dis import dis
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from math import atan2, cos, sin, sqrt, pi

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

# class VisionConnection:

#     def __init__(self):
#         # realsense vision connection
#         self.pipeline = rs.pipeline()
#         self.config = rs.config()

#     def rs_connection_set(self):

#         # Get device product line for setting a supporting resolution
#         pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
#         pipeline_profile = self.config.resolve(pipeline_wrapper)
#         device = pipeline_profile.get_device()
#         device_product_line = str(device.get_info(rs.camera_info.product_line))

#         found_rgb = False

#         for s in device.sensors:
#             if s.get_info(rs.camera_info.name) == 'RGB Camera':
#                 found_rgb = True
#                 break
            
#         if not found_rgb:
#             print("The demo requires Depth camera with Color sensor")
#             exit(0)
            
#         # self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#         self.config.enable_stream(rs.stream.depth)
        
#         if device_product_line == 'L500':
#             self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
#         else:
#             self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#         # Start streaming
#         self.profile = self.pipeline.start(self.config)

#         align_to = rs.stream.color  
#         self.align = rs.align(align_to)

#     def vision_frame_set(self):
#         frames = self.pipeline.wait_for_frames()

#         aligned_frames = self.align.process(frames)
    
#         depth_sensor = self.profile.get_device().first_depth_sensor()
#         self.depth_scale = depth_sensor.get_depth_scale()

#         self.aligned_depth_frame = aligned_frames.get_depth_frame()
#         color_frame = aligned_frames.get_color_frame()

#         # if not self.aligned_depth_frame or not color_frame:
#         #     continue
            
#         # depth_frame = frames.get_depth_frame()
#         # color_frame = frames.get_color_frame()

#         # if not depth_frame or not color_frame:
#         #     continue
            
#         self.depth_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
#         color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
#         depth_to_color_extrin = self.aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)
#         # print(depth_intrin)
#         # print(color_intrin)
#         # print(depth_to_color_extrin)
                    
#         # Convert images to numpy arrays
#         depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
#         color_image = np.asanyarray(color_frame.get_data())

#         # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
#         depth_colormap_dim = depth_colormap.shape
#         color_colormap_dim = color_image.shape
            
#         # vision_img = color_image

#         return color_image


def rpy_to_quaternion(rx, ry, rz):
   
    q = quaternion_from_euler(rx, ry, rz)

    return q

def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, mid_x, mid_y):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    # bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 51, 5)
    
    # cv2.imshow('Binary', bw)
    # cv2.waitKey(1)

    # dst = cv2.equalizeHist(gray)
    
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(bw, cv2.aruco_dict,parameters=parameters)
        # cameraMatrix=matrix_coefficients,
        # distCoeff=distortion_coefficients)
    # print(corners[0][0][0][1])
    # print(corners)

    rvec = np.zeros((1,1,3))
    tvec = np.zeros((1,1,3))

    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, matrix_coefficients,
                                                                       distortion_coefficients)
            
            (rvec - tvec).any()
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)
            # print(rvec)
            # print(tvec)
            # cv2.arrowedLine(frame, (mid_x, mid_y), (mid_x + 50, mid_y), (0,0,255), 2)
            # cv2.arrowedLine(frame, (mid_x, mid_y), (mid_x, mid_y + 50), (0,255,0), 2)

    return frame, rvec, tvec, ids

def obj_frame_drawAxis(img, p_, q_, color, scale):
    p = list(p_)
    q = list(q_)
    
    ## [visualization1]
    angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
    
    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
    
    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
    
    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
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
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
    # print(eigenvectors, eigenvalues)
    
    # Store the center of the object
    cntr = (int(mean[0,0]), int(mean[0,1]))
    # print(cntr)
    ## [pca]
    
    ## [visualization]

    angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians

    # Draw the principal components
    cv2.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
    # print(p1, p2)
    obj_frame_drawAxis(img, cntr, p1, (0, 0, 255), 1)
    # drawAxis(img, cntr, p2, (0, 255, 0), 1)
    
    # Label with the rotation angle
    label = "  Rotation Angle: " + str(-int(np.rad2deg(angle))) + " degrees"
    textbox = cv2.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
    cv2.putText(img, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
    
    return angle, cntr

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

def aruco_detection(vision_frame, depth_intrin, w, h):
    aruco_pose_Info = []
    aruco_dict_type = cv2.aruco.DICT_ARUCO_ORIGINAL
 
    k = np.array([[depth_intrin.fx, 0, depth_intrin.ppx], [0, depth_intrin.fy, depth_intrin.ppy], [0, 0, 1]])
    d = np.array([depth_intrin.coeffs[0], depth_intrin.coeffs[1], depth_intrin.coeffs[2], depth_intrin.coeffs[3], depth_intrin.coeffs[4]])

    # h, w = vision_frame.shape[:2]

    # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 1, (w, h))
            
    # dst = cv2.undistort(vision_frame, k, d, None, newcameramtx)
    # x, y, w1, h1 = roi
    # dst = dst[y:y+h1, x:x+w1]
    # vision_frame = dst

    mid_x, mid_y = w//2, h//2
    # print(mid_x, mid_y)
            
    aruco_img, rotation, translation, ids = pose_esitmation(vision_frame, aruco_dict_type, k, d, mid_x, mid_y)
    # print(type(ids))
    # rpy = np.array(rotation)
    # print(rotation.shape)
    # print(marker_index)

    rx = rotation[0][0][0]
    ry = rotation[0][0][1]
    rz = rotation[0][0][2]
    print(rx*180/pi, ry*180/pi, rz*180/pi)
            
    q_index = rpy_to_quaternion(rx, ry, rz)
        # print("Quaternion: ", q_)

    aruco_pose_Info = PoseStamped()
    aruco_pose_Info.header.frame_id = list('')
    aruco_pose_Info.pose.position.x = translation[0][0][0]
    aruco_pose_Info.pose.position.y = translation[0][0][1]
    aruco_pose_Info.pose.position.z = translation[0][0][2]
    aruco_pose_Info.pose.orientation.x = q_index[0]
    aruco_pose_Info.pose.orientation.y = q_index[1]
    aruco_pose_Info.pose.orientation.z = q_index[2]
    aruco_pose_Info.pose.orientation.w = q_index[3]

    return aruco_pose_Info, aruco_img #markerCornerPixel

def obj_detection(obj_image, depth_intrin, aligned_depth_frame, depth_scale):

    obj_pose_Info = []
    x_dist, y_dist, z_dist = [], [], []
    q_index = [], [], [], []
    # roi_y = int(roiStartPixel[1] - 10)
    # roi_x = int(roiStartPixel[0] - 10)
    # detecion_ROI = obj_image[roi_y: roi_y+200, roi_x: roi_x+400]
    # Convert image to grayscale
    gray = cv2.cvtColor(obj_image, cv2.COLOR_BGR2GRAY)

    # imgBlur = cv2.GaussianBlur(gray,(5,5),1)
    # imgCanny = cv2.Canny(imgBlur,100,100)
    # kernel = np.ones((5,5))
    # imgDial = cv2.dilate(imgCanny,kernel,iterations=3)
    # imgThre = cv2.erode(imgDial,kernel,iterations=2)
            
    # Convert image to binary
    # _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 2)
    # cv2.imshow('Binary', bw)
    # cv2.waitKey(1)

    # Find all the contours in the thresholded image
    contours, hierarchy = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    # print(contours[0].shape)

    obj_pose_Info = PoseStamped()
    if len(contours) == 0:
        obj_pose_Info.header = ''
        obj_pose_Info.pose = 0
    else:
        for i, c in enumerate(contours):
        
            # Calculate the area of each contour
            area = cv2.contourArea(c)
            
            # Ignore contours that are too small or too large
            if area < 3700 or 7000 < area:
                continue
            # print(contours[i].shape)
            # Draw each contour only for visualisation purposes
            cv2.putText(obj_image, '{:0.2f}'.format(area), tuple(c[0][0]), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
            cv2.drawContours(obj_image, contours, i, (0, 0, 255), 2)

            # Find the orientation of each shape
            angleR, centerPose = getOrientation(c, obj_image)

            dist = aligned_depth_frame.get_distance(centerPose[0], centerPose[1])
            depth_pixel = [centerPose[0], centerPose[1]]

            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dist*depth_scale)
            x_dist = round(depth_point[0]*1000, 4)
            y_dist = round(depth_point[1]*1000, 4)
            z_dist = round(depth_point[2]*1000, 4)
            rotation = [0, 0, angleR]

            q_index = rpy_to_quaternion(rotation[0], rotation[1], rotation[2])
            # print("Translation: ", x_dist, y_dist, z_dist)
            # print("Quaternion: ", q_index)

            
            obj_pose_Info.header.frame_id = "obj"
            obj_pose_Info.pose.position.x = x_dist
            obj_pose_Info.pose.position.y = y_dist
            obj_pose_Info.pose.position.z = z_dist
            obj_pose_Info.pose.orientation.x = q_index[0]
            obj_pose_Info.pose.orientation.y = q_index[1]
            obj_pose_Info.pose.orientation.z = q_index[2]
            obj_pose_Info.pose.orientation.w = q_index[3]
        
    return obj_pose_Info

def run_vision():
    bridge = CvBridge()
    rs_connection = VisionConnection()

    aruco_image_pub = rospy.Publisher('/aruco_detection_result_img', Image, queue_size=10)
    aruco_pose_Info_pub = rospy.Publisher('/aruco_pose_Info', PoseStamped, queue_size=10)

    obj_image_pub = rospy.Publisher('/ort_detection_result_img', Image, queue_size=10)
    obj_pose_Info_pub = rospy.Publisher('/obj_pose_Info', PoseStamped, queue_size=10)
    # rate = rospy.Rate(30)

    rs_connection.rs_connection_set()

    try:
        while not rospy.is_shutdown():
            img_frame, depth_color_frame = rs_connection.vision_frame_set()
            vision_frame, w, h = optimalFrame(img_frame, rs_connection.depth_intrin)
            cv2.imshow("Depth Map", depth_color_frame)
            cv2.waitKey(1)

            aruco_pose_Info, aruco_img = aruco_detection(vision_frame, rs_connection.depth_intrin, w, h)
            aruco_pose_Info_pub.publish(aruco_pose_Info)
            print(aruco_pose_Info)
            aruco_image_msg = bridge.cv2_to_imgmsg(aruco_img, "bgr8")
            aruco_image_pub.publish(aruco_image_msg)
            
            obj_pose_Info = obj_detection(vision_frame, rs_connection.depth_intrin, rs_connection.aligned_depth_frame, rs_connection.depth_scale)
            obj_pose_Info_pub.publish(obj_pose_Info)
            print(obj_pose_Info)
            obj_image_msg = bridge.cv2_to_imgmsg(vision_frame, "bgr8")
            obj_image_pub.publish(obj_image_msg)
    finally:
        rs_connection.pipeline.stop()

if __name__ == '__main__':
    rospy.init_node('integration_vision', anonymous=True)
    run_vision()
    rospy.spin()
