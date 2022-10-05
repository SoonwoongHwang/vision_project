import cv2
import measure_utlis as utlis
import numpy as np
import pyrealsense2 as rs
 
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
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

align_to = rs.stream.color  
align = rs.align(align_to)

scale = 3
wP = 210 *scale
hP= 297 *scale

while True:
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
            # print(depth_to_color_extrin)
                    
            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape
            
            vision_img = color_image

            aruco_dict_type = cv2.aruco.DICT_ARUCO_ORIGINAL
 
            k = np.array([[depth_intrin.fx, 0, depth_intrin.ppx], [0, depth_intrin.fy, depth_intrin.ppy], [0, 0, 1]])
            d = np.array([depth_intrin.coeffs[0], depth_intrin.coeffs[1], depth_intrin.coeffs[2], depth_intrin.coeffs[3], depth_intrin.coeffs[4]])
        

            # print(aruco_img.shape)
            h, w = vision_img.shape[:2]

            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 1, (w, h))
            
            dst = cv2.undistort(vision_img, k, d, None, newcameramtx)
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]
            vision_frame = dst
 
            imgContours , conts = utlis.getContours(vision_frame,minArea=50000,filter=4)
            # conts_ar = np.array(conts)
            # print(conts_ar.ndim)
            # print(conts_ar.shape)
            if len(conts) != 0:
                biggest = conts[0][2]
                # print(biggest)
                imgWarp = utlis.warpImg(vision_frame, biggest, wP,hP)
                imgContours2, conts2 = utlis.getContours(imgWarp,
                                                 minArea=2000, filter=4,
                                                 cThr=[50,50],draw = False)
                if len(conts) != 0:
                    for obj in conts2:
                        cv2.polylines(imgContours2,[obj[2]],True,(0,255,0),2)
                        nPoints = utlis.reorder(obj[2])
                        nW = round((utlis.findDis(nPoints[0][0]//scale,nPoints[1][0]//scale)/10),1)
                        nH = round((utlis.findDis(nPoints[0][0]//scale,nPoints[2][0]//scale)/10),1)
                        cv2.arrowedLine(imgContours2, (nPoints[0][0][0], nPoints[0][0][1]), (nPoints[1][0][0], nPoints[1][0][1]),
                                        (255, 0, 255), 3, 8, 0, 0.05)
                        cv2.arrowedLine(imgContours2, (nPoints[0][0][0], nPoints[0][0][1]), (nPoints[2][0][0], nPoints[2][0][1]),
                                        (255, 0, 255), 3, 8, 0, 0.05)
                        x, y, w, h = obj[3]
                        cv2.putText(imgContours2, '{}cm'.format(nW), (x + 30, y - 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5,
                                    (255, 0, 255), 2)
                        cv2.putText(imgContours2, '{}cm'.format(nH), (x - 70, y + h // 2), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5,
                                    (255, 0, 255), 2)
                cv2.imshow('A4', imgContours2)
 
            img = cv2.resize(vision_frame,(0,0),None,0.5,0.5)
            cv2.imshow('Original',vision_frame)
            cv2.waitKey(1)