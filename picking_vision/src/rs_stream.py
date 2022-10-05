import pyrealsense2 as rs
import cv2
import numpy as np

class VisionConnection:

    def __init__(self):
        # realsense vision connection
        self.pipeline = rs.pipeline()
        self.config = rs.config()

    def rs_connection_set(self):

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
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
            
        # self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.depth)
        
        if device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        align_to = rs.stream.color  
        self.align = rs.align(align_to)

    def vision_frame_set(self):
        colorizer = rs.colorizer()
        frames = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frames)
    
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        self.aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # if not self.aligned_depth_frame or not color_frame:
        #     continue
            
        # depth_frame = frames.get_depth_frame()
        # color_frame = frames.get_color_frame()

        # if not depth_frame or not color_frame:
        #     continue
            
        self.depth_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = self.aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)
        # print(depth_intrin)
        # print(color_intrin)
        # print(depth_to_color_extrin)
                    
        # Convert images to numpy arrays
        depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # grey_color = 153
        # depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        # bg_removed = np.where((depth_image_3d > 1 / self.depth_scale) | (depth_image_3d <= 0), grey_color, color_image)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap = np.asanyarray(colorizer.colorize(self.aligned_depth_frame).get_data())
        # depth_color_image = np.hstack((bg_removed, depth_colormap))
        # depth_color_image = np.hstack((color_image, depth_colormap))

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape
            
        # vision_img = color_image

        return color_image, depth_colormap