#!/usr/bin/env python3

import rospy
import math

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# Import Sphere Parameters
from robot_vision_lectures.msg import SphereParams

# Imports for transformations:
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs

# variables used to get pre-transformation sphere data:
raw_x = 0
raw_y = 0
raw_z = 0
radius = 0


def get_sphere_data(data):

	global raw_x
	global raw_y
	global raw_z
	global radius
	
	raw_x = data.xc
	raw_y = data.yc
	raw_z = data.zc
	radius = data.radius
	
def transform_coords(raw_x, raw_y, raw_z, radius):
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	flag = False

	q_rot = Quaternion()	
	while not flag:
	
		# try getting the most update transformation between the camera frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			flag = True
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
	
	# Define a point in the camera frame:
	pt_camera = tf2_geometry_msgs.PointStamped()
	pt_camera.header.frame_id = 'camera_color_optical_frame'
	pt_camera.header.stamp = rospy.get_rostime()
	
	pt_camera.point.x = raw_x
	pt_camera.point.y = raw_y
	pt_camera.point.z = raw_z
	
	# Convert the 3D point to the base frame coordinates:
	pt_base = tfBuffer.transform(pt_camera,'base', rospy.Duration(1.0))
	
	# Get XYZ coordinates from new frame:
	x = pt_base.point.x
	y = pt_base.point.y
	z = pt_base.point.z
	
	# Return XYZ coords:
	return x, y, z, radius
		
		

# define a function that defines a new point
def add_point(plan, linX, linY, linZ, anX, anY, anZ):
		plan_point = Twist()
		
		plan_point.linear.x = linX
		plan_point.linear.y = linY
		plan_point.linear.z = linZ
		plan_point.angular.x = anX
		plan_point.angular.y = anY
		plan_point.angular.z = anZ
		
		plan.points.append(plan_point)
		

# Callback function for getting current robot position:

current_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
def get_current_pos(data):
	global current_pos
	
	current_pos[0] = data.linear.x
	current_pos[1] = data.linear.y
	current_pos[2] = data.linear.z
	current_pos[3] = data.angular.x
	current_pos[4] = data.angular.y
	current_pos[5] = data.angular.z

# Callback function for enabling/disabling ball planning:
planning = False
def plan_bool(data):
	global planning
	planning = data.data

# Callback function to check if the ball is being tracked:
tracking = False
def track_bool(data):
	global tracking
	tracking = data.data

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('custom_planner', anonymous = True)
	
	# Add a subscriber for the sphere parameters:
	rospy.Subscriber('sphere_params', SphereParams, get_sphere_data)
	
	# Add a subscriber for the robot's curent position:
	rospy.Subscriber('/ur5e/toolpose', Twist, get_current_pos)
	
	# Add a subscriber for boolean value:
	rospy.Subscriber('/BallPlan', Bool, plan_bool)
	
	# Add a subscriber for boolean value:
	rospy.Subscriber('/TrackBall', Bool, track_bool)
	
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	delay = 0
	while not rospy.is_shutdown():
	
		if planning and tracking: # Check to make sure tracking and planning are enabled
		# Each time tracking is re-enabled, a new plan is defined with updated starting position and ball position
			
			print("---\nGenerating new plan...")
			
			valid = False
			while not valid:
			
				if 0 not in [raw_x, raw_y, raw_z, radius]: # Ensures that the results are satisfactory before building plan
					valid = True
					
				# Conduct transformation:
				new_coords = transform_coords(raw_x, raw_y, raw_z, radius)
						
			
			print("---\nBall values before transformation:")
			print("x: {}, y: {}, z: {}, radius: {}".format(raw_x, raw_y, raw_z, radius))
			
			# Define transformed coordinates:
			x = new_coords[0]
			y = new_coords[1]
			z = new_coords[2]
			r = new_coords[3]
			
			print("Ball values after transformation:")
			print("x: {}, y: {}, z: {}, radius: {}".format(x, y, z, r))
			print("---\nInitializing motion...")
			
			motion = Plan() # New plan
			
			# First point, initial position:
			add_point(motion, current_pos[0], current_pos[1], current_pos[2], current_pos[3], current_pos[4], current_pos[5])
			
			# Second point, directly over ball:
			add_point(motion, x, y, 0.3, current_pos[3], current_pos[4], current_pos[5])
			
			# Third point, straignt down to pick up ball:
			add_point(motion, x, y, (z)+r, current_pos[3], current_pos[4], current_pos[5])
			
			# Fourth point, straight back up:
			add_point(motion, x, y, 0.3, current_pos[3], current_pos[4], current_pos[5])
			
			# Fifth point, above drop point:
			add_point(motion, 0.3, -0.35, 0.3, current_pos[3], current_pos[4], current_pos[5])
			
			# Sixth point, straight down to drop ball:
			add_point(motion, 0.3, -0.35, (z)+r, current_pos[3], current_pos[4], current_pos[5])
			
			# Seventh point, back up:
			add_point(motion, 0.3, -0.35, 0.3, current_pos[3], current_pos[4], current_pos[5])
			
			while planning: # This will stop publishing the plan when tracking is disabled
				
				# publish the plan
				plan_pub.publish(motion)
			
				# wait for 0.1 seconds until the next loop and repeat
				
				loop_rate.sleep()
		
		else: # Print a message to the user if tracking is not enabled:
	
			if delay == 0:
				if not planning:
					print("---\nMotion Planning not enabled. Change topic \'/BallPlan\' (type std_msgs/Bool) in rqt_gui.")
				if not tracking:
					print("---\nBall tracking not enabled. Change topic \'/TrackBall\' (type std_msgs/Bool) in rqt_gui.")
				delay = 10
			else:
				delay -= 1 # Less overwhelming stream of messages
				loop_rate.sleep()
		
	
