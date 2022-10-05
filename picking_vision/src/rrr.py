import cv2 as cv
from math import atan2, cos, sin, sqrt, pi
import numpy as np
from PIL import Image

# Load the image
input_img = Image.open('/home/hsw/catkin_ws/src/picking_vision/src/gun_input_img.jpg')
img_resize = input_img.resize((640, 480))
img_resize.save('/home/hsw/catkin_ws/src/picking_vision/src/gun_input_img_re.jpg')
img = cv.imread("/home/hsw/catkin_ws/src/picking_vision/src/gun_input_img_re.jpg")
 
# Was the image there?
if img is None:
  print("Error: File not found")
  exit(0)
 

 
# Convert image to grayscale
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('Input Image', gray)
cv.waitKey(0)
# cv.destroyWindow()
# Convert image to binary
_, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)


contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
# cv.imshow('result', contours)
# print(contours, _)
# cv.waitKey(0)

for i, c in enumerate(contours):
  print(c)
  area = cv.contourArea(c)
 
  # Ignore contours that are too small or too large
  # if area < 3700 or 100000 < area:
  #   continue
 
  # Draw each contour only for visualisation purposes
  cv.drawContours(img, contours, i, (0, 0, 255), 2)

  pts = c

  sz = len(pts)
  print(sz)
  
  data_pts = np.empty((sz, 2), dtype=np.float64)


# cv.imshow('Output Image', contours)
# cv.waitKey(0)
# cv.destroyAllWindows()