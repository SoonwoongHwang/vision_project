#!/usr/bin/env python3


from email.headerregistry import MessageIDHeader
import cv2
import sys
from math import atan2, cos, sin, sqrt, pi
import numpy as np
from PIL import Image as pImage
from torch import rad2deg

from dis import dis
import rospy
import message_filters
import pyrealsense2 as rs

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
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
    drawAxis(img, cntr, p1, (0, 0, 255), 1)
    # drawAxis(img, cntr, p2, (0, 255, 0), 1)
    
    # Label with the rotation angle
    label = "  Rotation Angle: " + str(-int(np.rad2deg(angle))) + " degrees"
    textbox = cv2.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
    cv2.putText(img, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
    
    return angle, cntr

def callback(rgb_msg, camera_info, depth_camera_info, depthImg_msg):
    bridge = CvBridge()
  
    cv_image = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    cv_depth_image = bridge.imgmsg_to_cv2(depthImg_msg, depthImg_msg.encoding)
    print(cv_image.shape)
    print(cv_depth_image.shape)

    depth_intrinsics = rs.intrinsics()
    depth_intrinsics.width = depth_camera_info.width
    depth_intrinsics.height = depth_camera_info.height
    depth_intrinsics.ppx = depth_camera_info.K[2]
    depth_intrinsics.ppy = depth_camera_info.K[5]
    depth_intrinsics.fx = depth_camera_info.K[0]
    depth_intrinsics.fy = depth_camera_info.K[4]

    if depth_camera_info.distortion_model == 'plumb_bob':
        depth_intrinsics.model = rs.distortion.brown_conrady
    elif depth_camera_info.distortion_model == 'equidistant':
        depth_intrinsics.model = rs.distortion.kannala_brandt4
    depth_intrinsics.coeffs = [i for i in depth_camera_info.D]
    # print(depth_intrinsics)

    k = np.array(camera_info.K).reshape([3, 3])
    d = np.array(camera_info.D)

    h, w = cv_image.shape[:2]

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 1, (w, h))
    # print(newcameramtx)
    # print(k)
    dst = cv2.undistort(cv_image, k, d, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv_frame = dst

    pix = (depthImg_msg.width//2, depthImg_msg.height//2)

    ## Histogram_start
    # cv_frame_ycrcb = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2YCrCb)
    # ycrcb_planes = cv2.split(cv_frame_ycrcb)

    # ycrcb_planes[0] = cv2.equalizeHist(ycrcb_planes[0])

    # dst_ycrcb = cv2.merge(ycrcb_planes)
    # cv_dst = cv2.cvtColor(dst_ycrcb, cv2.COLOR_YCrCb2BGR)
    ## Histogram_end

    # Convert image to grayscale
    gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
    # matchgray = cv.cvtColor(match_shape, cv.COLOR_BGR2GRAY)
 
    # Convert image to binary
    _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    # _, mbw = cv.threshold(matchgray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
  
    # Find all the contours in the thresholded image
    contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    # mcontours, _ = cv.findContours(mbw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

    # matches = []
    for i, c in enumerate(contours):

        # Calculate the area of each contour
        area = cv2.contourArea(c)
  
        # Ignore contours that are too small or too large
        if area < 3700 or 100000 < area:
          continue

        # match = cv.matchShapes(mcontours[0], c, cv.CONTOURS_MATCH_I2, 0.0)
        # matches.append((match, c))
        # cv.putText(vision_img, '{:0.2f}'.format(match), tuple(c[0][0]), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
        # matches.sort(key=lambda x:x[0])
        # Draw each contour only for visualisation purposes
        cv2.putText(cv_frame, '{:0.2f}'.format(area), tuple(c[0][0]), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
        cv2.drawContours(cv_frame, contours, i, (0, 0, 255), 2)
  
        # Find the orientation of each shape
        angleR, centerPose = getOrientation(c, cv_frame)

        # dist = aligned_depth_frame.get_distance(centerPose[0], centerPose[1])
        depth = cv_depth_image[pix[1], pix[0]]
        depth_pixel = [centerPose[0], centerPose[1]]
        # print(depth_intrin)
        # print(depth_pixel)
        # print(dist)
        # print(depth_scale)
        # print(dist*depth_scale)
        depth_point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, depth_pixel, depth)
        x_dist = round(depth_point[0]*1000, 4)
        y_dist = round(depth_point[1]*1000, 4)
        z_dist = round(depth_point[2]*1000, 4)
        # print(depth_point)

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

        # oriented_obj_Info_pub.publish(oriented_obj_Info)

        # print(oriented_obj_Info)
    
    rosVision_pub = rospy.Publisher("/opencvToros_img", Image, queue_size=10)

    cvimgToros = bridge.cv2_to_imgmsg(cv_frame, "bgr8")
    rosVision_pub.publish(cvimgToros)
    # cv2.imshow("Test", cv_image)
    # cv2.waitKey(1)


def ros_vision_connection_node():
    rospy.init_node('ros_vision_connection', anonymous=True)
    image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    info_sub = message_filters.Subscriber("/camera/color/camera_info", CameraInfo)
    depth_info_sub = message_filters.Subscriber("/camera/depth/camera_info", CameraInfo)
    depth_image_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub, depth_info_sub, depth_image_sub], 10, 0.2)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    ros_vision_connection_node()

