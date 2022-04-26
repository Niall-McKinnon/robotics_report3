#!/usr/bin/env python3
import math
import rospy
import numpy as np
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from std_msgs.msg import Bool

# Define global variables, these are used to build A and B matricies:
b_data = []
a_data = []
received = False

# Callback function to get ball data from publisher:
def get_ball_data(data):
	
	# Access global variables:
	global received
	global b_data
	global a_data
	
	# Redefine lists as empty for each new dataset:
	b_data = []
	a_data = []
	
	# Build data foor A and B:
	for point in data.points:
		
		b_data.append((point.x)**2 + (point.y)**2 + (point.z)**2)
		
		a_data.append([2*point.x, 2*point.y, 2*point.z, 1])
	
	received = True
	

# Function to filter the data:
def filter_data(data, fil_out, fil_gain):
	
	# Initialise values before calculations:
	fil_in = data
	
	# Filter equation:
	fil_out = (fil_gain*fil_in) + (1 - fil_gain)*fil_out
	
	return fil_out

# Callback for boolean switch:
tracking = None
def track_bool(data):
	global tracking
	tracking = data.data

if __name__ == '__main__':
	
	# Initialize the node:
	rospy.init_node('sphere_fit', anonymous = True)
	
	# Add a subscriber for the XYZ data:
	sphere_data = rospy.Subscriber('xyz_cropped_ball', XYZarray, get_ball_data)
	
	# Add a subscriber for boolean value:
	rospy.Subscriber('/TrackBall', Bool, track_bool)
	
	# Define publisher:
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 1)
	
	# This was a seperate pub that I used to compare the initial and filtered data in rqt_plot:
	# filter_pub = rospy.Publisher('/filtered_params', SphereParams, queue_size = 1)
	
	# Set loop frequency:
	rate = rospy.Rate(10)
	
	# Initialize variables for filter equation:
	x_fil_out = 0.0
	y_fil_out = 0.0
	z_fil_out = 0.0
	r_fil_out = 0.0
	
	# Gains are easily adjustable:
	point_gain = 0.0005
	radius_gain = 0.005
	
	delay = 0
	first = True
	off_print = True
	while not rospy.is_shutdown():
		# if tracking:

		if received: # if tracking:
			
			# Define the A matrix:
			A = np.array(a_data)
			
			# Define the B matrix:
			B = np.array([b_data]).T
			
			# Check validity of data to avoid errors:
			if A.shape[0] == B.shape[0] and len(A.shape) == 2 and len(B.shape) == 2:
			
				# Calculate P:
				P = np.linalg.lstsq(A, B, rcond=None)[0]
			
				# Get sphere params from P:
				xc = P[0]
				yc = P[1]
				zc = P[2]
				r = math.sqrt(P[3] + xc**2 + yc**2 + zc**2)
				
				if tracking: # If tracking is enabled, filter the data
				
					off_print = True  # This is a simple flag used for printing a message if the filter is disabled
				
					# If this is the first time data is received, do not filter:
					if first:
						x_fil_out = xc
						y_fil_out = yc
						z_fil_out = zc
						r_fil_out = r
						
						print("---\nBall Filter is ENABLED.")
						# The first filter input is set to the initial coordinates
						first = False
						
					else:
						# Get filtered data:
						xc = filter_data(xc, x_fil_out, point_gain)
						x_fil_out = xc
						yc = filter_data(yc, y_fil_out, point_gain)
						y_fil_out = yc
						zc = filter_data(zc, z_fil_out, point_gain)
						z_fil_out = zc
						r = filter_data(r, r_fil_out, radius_gain)
						r_fil_out = r
						
				else:
					if off_print:
						print("---\nBall filter is DISABLED.")
						off_print = False
					first = True  # Reset first to True for when filter is re-initialized
				
				# This was for the seperate filtered data publisher
				# I have kept it commented here for ease of access later in case I need to access both datasets
				
				#filtered_data = SphereParams()
				#filtered_data.xc = Fxc
				#filtered_data.yc = Fyc
				#filtered_data.zc = Fzc
				#filtered_data.radius = Fr
				#filter_pub.publish(filtered_data)
				
				# Declare variable for publisher:
				sphere_data = SphereParams()
				
				# Add sphere params to publisher:
				sphere_data.xc = xc
				sphere_data.yc = yc
				sphere_data.zc = zc
				sphere_data.radius = r
				
				# Publish messge:
				sphere_pub.publish(sphere_data)
				
