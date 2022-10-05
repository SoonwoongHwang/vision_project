#!/usr/bin/env python3

#import appropriate python modules to the program
import numpy as np
import cv2
from matplotlib import pyplot as plt
from picking_vision.msg import ObjInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import pyrealsense2 as rs
import torch
import rospy

# initialization
global input_mode, initialize_mode
global box_pts, frame
global object_pose_Info

global record_num, record_mode
global t1, t2, t3, r1, r2, r3, rx, ry, rz
global rs_intrinsic_param, rs_distortion_param
# global orb

def rpy_to_quaternion(rotation):

    q = quaternion_from_euler(rotation[0], rotation[1], rotation[2])

    return q

# callback function for selecting object by clicking 4-corner-points of the object
def select_object(event, x, y, flags, param):
    global box_pts, frame
    if input_mode and event == cv2.EVENT_LBUTTONDOWN and len(box_pts) < 4:
        box_pts.append([x, y])
        frame = cv2.circle(frame, (x, y), 4, (0, 255, 0), 2)

# selecting object by clicking 4-corner-points
def select_object_mode():
    global input_mode, initialize_mode
    input_mode = True
    
    frame_static = frame.copy()

    while len(box_pts) < 4:
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
    
    initialize_mode = True
    input_mode = False

# setting the boundary of reference object
def set_boundary_of_reference(box_pts):
    
    ### upper bound ###
    if box_pts[0][1] < box_pts[1][1]:
        upper_bound = box_pts[0][1]
    else:
        upper_bound = box_pts[1][1]
    
    ### lower bound ###
    if box_pts[2][1] > box_pts[3][1]:
        lower_bound = box_pts[2][1]
    else:
        lower_bound = box_pts[3][1]
    
    ### left bound ###
    if box_pts[0][0] < box_pts[2][0]:
        left_bound = box_pts[0][0]
    else:
        left_bound = box_pts[2][0]
    
    ### right bound ###
    if box_pts[1][0] > box_pts[3][0]:
        right_bound = box_pts[1][0]
    else:
        right_bound = box_pts[3][0]
        
    upper_left_point = [0,0]
    upper_right_point = [(right_bound-left_bound),0]
    lower_left_point = [0,(lower_bound-upper_bound)]
    lower_right_point = [(right_bound-left_bound),(lower_bound-upper_bound)]
    
    pts2 = np.float32([upper_left_point, upper_right_point, lower_left_point, lower_right_point])
    
    # display dimension of reference object image to terminal
    print(pts2)
    
    return pts2, right_bound, left_bound, lower_bound, upper_bound

# doing perspective transform to reference object
def input_perspective_transform(box_pts, pts2, right_bound, left_bound, lower_bound, upper_bound):
    global object_orb
    pts1 = np.float32(box_pts)
    M = cv2.getPerspectiveTransform(pts1, pts2)
    img_object = cv2.warpPerspective(frame,M,((right_bound-left_bound),(lower_bound-upper_bound)))
    return cv2.cvtColor(img_object, cv2.COLOR_BGR2GRAY)

# feature detection and description using ORB
def orb_feature_descriptor(img_object, orb):
    kp1, des1 = orb.detectAndCompute(img_object,None)
    kp2, des2 = orb.detectAndCompute(frame,None)
    return kp1, des1, kp2, des2

# feature matching using Brute Force
def brute_force_feature_matcher(kp1, des1, kp2, des2):
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    return sorted(matches, key = lambda x:x.distance)

# finding homography matrix between reference and image frame
def find_homography_object(kp1, kp2, matches):
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    return M, mask

# applying homography matrix as inference of perpective transformation
def output_perspective_transform(img_object, M):
    h,w = img_object.shape
    corner_pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    center_pts = np.float32([ [w/2,h/2] ]).reshape(-1,1,2)
    corner_pts_3d = np.float32([ [-w/2,-h/2,0],[-w/2,(h-1)/2,0],[(w-1)/2,(h-1)/2,0],[(w-1)/2,-h/2,0] ])###
    corner_camera_coord = cv2.perspectiveTransform(corner_pts,M)###
    center_camera_coord = cv2.perspectiveTransform(center_pts,M)
    return corner_camera_coord, center_camera_coord, corner_pts_3d, center_pts

# solving pnp using iterative LMA algorithm
def iterative_solve_pnp(object_points, image_points):
    image_points = image_points.reshape(-1,2)
    retval, rotation, translation = cv2.solvePnP(object_points, image_points, rs_intrinsic_param, rs_distortion_param)
    return rotation, translation

# drawing box around object
def draw_box_around_object(dst):
    return cv2.polylines(frame, [np.int32(dst)],True,255,3)
    
# recording sample data
def record_samples_data(translation, rotationd):
    translation_list = translation.tolist()
    rotationd_list = rotationd.tolist()
    
    t1.append(translation_list[0])
    t2.append(translation_list[1])
    t3.append(translation_list[2])
    
    r1.append(rotationd_list[0])
    r2.append(rotationd_list[1])
    r3.append(rotationd_list[2])
    
# computing and showing recorded data to terminal
def showing_recorded_data_to_terminal(t1, t2, t3, r1, r2, r3):
    
    # convert to numpy array
    t1 = np.array(t1)
    t2 = np.array(t2)
    t3 = np.array(t3)
    
    r1 = np.array(r1)
    r2 = np.array(r2)
    r3 = np.array(r3)
    
    # print mean and std of the data to terminal
    print("mean t1", np.mean(t1))
    print("std t1", np.std(t1)) 
    print("") 
    print("mean t2", np.mean(t2)) 
    print("std t2", np.std(t2)) 
    print("") 
    print("mean t3", np.mean(t3)) 
    print("std t3", np.std(t3)) 
    print("") 
    print("") 
    print("mean r1", np.mean(r1)) 
    print("std r1", np.std(r1))
    print("")
    print("mean r2", np.mean(r2))
    print("std r2", np.std(r2))
    print("")
    print("mean r3", np.mean(r3))
    print("std r3", np.std(r3))
    print("")
    print("#####################")
    print("")

# showing object position and orientation value to frame
def put_position_orientation_value_to_frame(translation, rotation):
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    cv2.putText(frame,'position(m)',(10,30), font, 0.7,(0,255,0),1,cv2.LINE_AA)
    cv2.putText(frame,'x:'+str(np.round(translation[0],3)),(250,30), font, 0.7,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(frame,'y:'+str(np.round(translation[1],3)),(350,30), font, 0.7,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(frame,'z:'+str(np.round(translation[2],3)),(450,30), font, 0.7,(0,0,255),2,cv2.LINE_AA)
    
    cv2.putText(frame,'orientation(degree)',(10,60), font, 0.7,(0,255,0),1,cv2.LINE_AA)
    cv2.putText(frame,'x:'+str(np.round(rotation[0],2)),(250,60), font, 0.7,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(frame,'y:'+str(np.round(rotation[1],2)),(350,60), font, 0.7,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(frame,'z:'+str(np.round(rotation[2],2)),(450,60), font, 0.7,(0,0,255),2,cv2.LINE_AA)
    
    return frame


def vision_recognized_obj_publisher():
    ############
    ### Main ###
    ############
    bridge = CvBridge()
    rospy.init_node('vision_recognized_object', anonymous=False)
    image_pub = rospy.Publisher('obb_frame_img', Image, queue_size=10)
    object_pose_Info_pub = rospy.Publisher('/object_pose_Info', PoseStamped, queue_size=10)
    rate = rospy.Rate(30)

    # Yolov5 model load
    # model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

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

    frames = pipeline.wait_for_frames()

    # initialization
    global input_mode, initialize_mode
    input_mode = False
    initialize_mode = False
    track_mode = False
    global box_pts, frame
    box_pts = []

    global record_num, record_mode
    record_num = 0
    record_mode = False

    global t1, t2, t3, r1, r2, r3
    t1, t2, t3, r1, r2, r3 = [], [], [], [], [], []

    global rs_intrinsic_param
    global rs_distortion_param
    rs_intrinsic_param = np.array([[514.04093664*0.55, 0., 320*1.03], [0., 514.8747658*0.55, 240*1.13], [0., 0., 1.]])
    rs_distortion_param = np.array([2.68661165e-01, -1.31720458e+00, -3.22098653e-03, -1.11578383e-03, 2.44470018e+00])

    # rs_intrinsic_param = np.array([[638.297878*0.46, 0, 641.520437*0.46], [0, 638.68774*0.46, 365.744002*0.71], [0, 0, 1]])
    # rs_distortion_param = np.array([-0.044848, 0.035995, -0.000144, -0.001623, 0])

    # global orb
    orb = cv2.ORB_create()

    cv2.namedWindow("frame")
    # cv2.resizeWindow("frame", 1280, 720)

    cv2.setMouseCallback("frame", select_object)

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
        # if depth_colormap_dim != color_colormap_dim:
        #     resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        #     frame = np.hstack((resized_color_image, depth_colormap))
        # else:
        #     frame = np.hstack((color_image, depth_colormap))

        # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', frame)

        # global frame 
        frame = color_image
        # results = model(frame)
        # results_img = results.render()
        # results_img = model(frame).render()
        # results_img = np.asarray(results_img)
        # print(results_img)
        
        k = cv2.waitKey(1) & 0xFF
        
        # press i to enter input mode
        if k == ord('i'):
            
            # select object by clicking 4-corner-points
            select_object_mode()
            
            # set the boundary of reference object
            pts2, right_bound, left_bound, lower_bound, upper_bound = set_boundary_of_reference(box_pts)
            
            # do perspective transform to reference object
            img_object = input_perspective_transform(box_pts, pts2, right_bound, left_bound, lower_bound, upper_bound)
            
            track_mode = True
        
        # track mode is run immediately after user selects 4-corner-points of object
        if track_mode is True:
            # feature detection and description
            kp1, des1, kp2, des2 = orb_feature_descriptor(img_object, orb)
            
            # feature matching
            matches = brute_force_feature_matcher(kp1, des1, kp2, des2)
            
            # find homography matrix
            M, mask = find_homography_object(kp1, kp2, matches)
            
            # apply homography matrix using perspective transformation
            corner_camera_coord, center_camera_coord, object_points_3d, center_pts = output_perspective_transform(img_object, M)
            
            # solve pnp using iterative LMA algorithm
            rotation, translation = iterative_solve_pnp(object_points_3d, corner_camera_coord)
            # print("Translation: ", translation)
            # print("Rotation: ", rotation)
            # print("x", translation[0])
            # print("y", translation[1])
            # print("z", translation[2])
            # print("rx", rotation[0])
            # print("ry", rotation[1])
            # print("rz", rotation[2])
            # convert to centimeters
            translation = translation *.001    # (40./53.) *
            
            q_ = rpy_to_quaternion(rotation)
            # print("Quaternion: ", q_)

            object_pose_Info = PoseStamped()
            object_pose_Info.pose.position.x = translation[0]
            object_pose_Info.pose.position.y = translation[1]
            object_pose_Info.pose.position.z = translation[2]
            object_pose_Info.pose.orientation.x = q_[0]
            object_pose_Info.pose.orientation.y = q_[1]
            object_pose_Info.pose.orientation.z = q_[2]
            object_pose_Info.pose.orientation.w = q_[3]

            object_pose_Info_pub.publish(object_pose_Info)
            # print(object_pose_Info)
            
            # convert to d`egree
            rotationd = rotation * 180./np.pi
            
            # press r to record 50 sample data and calculate its mean and std
            if k == ord("r"):
                record_mode = True
                
            if record_mode is True :
                record_num = record_num + 1
                
                # record 50 data
                record_samples_data(translation, rotationd)
                
                if record_num == 50:
                    record_mode = False
                    record_num = 0
            
                    # compute and show recorded data
                    # showing_recorded_data_to_terminal(t1, t2, t3, q_[0], q_[1], q_[2], q_[3])
                    showing_recorded_data_to_terminal(t1, t2, t3, r1, r2, r3)
                    
                    # reset the data after 50 iterations
                    t1, t2, t3, r1, r2, r3 = [], [], [], [], [], []
                    # t1, t2, t3, q_[0], q_[1], q_[2], q_[3] = [], [], [], [], [], []
            
            # draw box around object
            frame = draw_box_around_object(corner_camera_coord)
            
            # show object position and orientation value to frame
            frame = put_position_orientation_value_to_frame(translation, rotation)
        
        cv2.imshow("frame", frame)
        # print(frame_view)

        imgage_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        image_pub.publish(imgage_msg)

        # break when user pressing ESC
        if k == 27:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
	vision_recognized_obj_publisher()