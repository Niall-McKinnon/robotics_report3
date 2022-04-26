#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


img_received = False
# Define a 720x1280 3-channel image with all pixels equal to zero:
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")


# Get the image message:
def get_image(ros_img):
	global rgb_img
	global img_received
	# Convert to opencv image:
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# Raise flag:
	img_received = True

	
if __name__ == '__main__':
	# Define the node and subcribers and publishers:
	rospy.init_node('detect_ball', anonymous = True)
	# Define a subscriber to ream images:
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
	# Define a publisher to publish images:
	mask_pub = rospy.Publisher('/ball_2D', Image, queue_size = 1) # Black and white mask
	overlay_pub = rospy.Publisher('/ball_overlay', Image, queue_size = 1) # Original video with circle overlay
	
	# Set the loop frequency:
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			
			# Convert image to HSV colorspace:
			hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
			
			# Define ranges of the mask:
			#lower_yellow_hsv = np.array([20,10,1])
			#upper_yellow_hsv = np.array([60,255,255])
			
			lower_yellow_hsv = np.array([0,80,230])
			upper_yellow_hsv = np.array([40,255,255])
			
			# Use mask to filter the image:
			mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
			
			# Define grayscale image for circle detection:
			gray = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2GRAY)
			gray = cv2.medianBlur(gray, 5)
			
			# Detect circles:
			rows = gray.shape[0]
			circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows/8, param1=100, param2=36, minRadius=50, maxRadius=0)
			
			# Draw circles on both mask and rgb image:
			if circles is not None:
				circles = np.uint16(np.around(circles))
				for i in circles[0, :]:
				
					# Center Point:
					center = (i[0], i[1])
					cv2.circle(rgb_img, center, 1, (0, 100, 100), 3)
					
					# Outline:
					radius = i[2]
					cv2.circle(rgb_img, center, radius, (255, 0, 0), 3)
			
			# Crop image mask to remove accidental detection:
			blank_img = np.zeros((720, 1280, 1), dtype = "uint8")
			cv2.rectangle(blank_img, (120, 120), (1180, 600), 255, -1)
			mask = cv2.bitwise_and(blank_img, mask)
			
			# Convert images to ros msg and publish:
			
			# Black and white mask:
			mask_msg = CvBridge().cv2_to_imgmsg(mask, encoding="mono8")
			mask_pub.publish(mask_msg)
			
			# Original video with circle overlay:
			overlay_msg = CvBridge().cv2_to_imgmsg(rgb_img, encoding="rgb8")
			overlay_pub.publish(overlay_msg)
			
		# Pause until the next iteration:
		rate.sleep()
