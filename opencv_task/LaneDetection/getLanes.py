#!/usr/bin/python3

# imports
import numpy as np
import cv2
import imutils
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math

# load the input image (whose path was supplied via command line
# argument) and display the image to our screen
image = cv2.imread("task1_pic.jpeg")

print("Original Image")
print("Press any key to continue")
cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
height = image.shape[0]
width = image.shape[1]

# thresholding using HSL image, for white and yellow lanes
image_hsl = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

white_low = (0, 200, 0)
white_high = (255, 255, 255)
yellow_low = (20, 100, 100)
yellow_high = (40, 255, 255)

# return all white areas and yellow areas
white_lanes = cv2.inRange(image_hsl, white_low, white_high)
yellow_lanes = cv2.inRange(image_hsl, yellow_low, yellow_high)

lanes_total = cv2.bitwise_or(white_lanes, yellow_lanes)

print("White Lanes Only")
print("Press any key to continue")
cv2.imshow("White Lanes", white_lanes)
cv2.waitKey(0)

print("Yellow Lanes Only")
print("Press any key to continue")
cv2.imshow("Yellow Lanes", yellow_lanes)
cv2.waitKey(0)

print("All lanes")
print("Press any key to continue")
cv2.imshow("All Lanes", lanes_total)
cv2.waitKey(0)

cv2.destroyAllWindows()

# Canny edge detection
blur = cv2.GaussianBlur(lanes_total, (7, 7), 0)
canny = cv2.Canny(blur, 50, 150)

print("Canny Edge Detection")
print("Press any key to continue")
cv2.imshow("Canny Edges", canny)
cv2.waitKey(0)

cv2.destroyAllWindows()

#Selecting region of interest - the triangle-like shape the road takes from observer to horizon

mask2 = np.zeros_like(canny)                              # (height, width)
myROI = [(0, height), (0, int(height * 0.9)), (int(width*0.5), int(height * 0.6)), (width, int(height * 0.9)), (width, height)]  # (x, y)
cv2.fillPoly(mask2, [np.array(myROI)], 255)

road_lanes = cv2.bitwise_and(mask2, canny)

print("Selecting Region of Interest")
print("Press any key to continue")
cv2.imshow("RoI selection", road_lanes)
cv2.waitKey(0)

cv2.destroyAllWindows()

# Hough line transform

# Probabilistic version
lines = cv2.HoughLinesP(road_lanes, 1, np.pi/180, 25, minLineLength = 10, maxLineGap = 10)
image2 = image

for line in lines:
    x1,y1,x2,y2 = line[0]
    y3 = height
    x3 = ((x2-x1)/(y2-y1))*(y3-y1) + x1
    y3 = int(y3)
    x3 = int(x3)
    y4 = height * 0.6
    x4 = ((x2-x1)/(y2-y1))*(y4-y1) + x1
    y4 = int(y4)
    x4 = int(x4)
    cv2.line(image2,(x3,y3),(x4,y4),(0,0,255),10)

print("Final Result")
print("Press any key to continue")
cv2.imshow("Hough Lines - Final Answer", image2)
cv2.waitKey(0)
cv2.destroyAllWindows()
