
from __future__ import print_function

import sys
import os
import math
import time
import threading
import logging
import traceback

import rospy
import cv2

import numpy as np
import miro2 as miro

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import UInt8, UInt32, String, UInt16MultiArray, Float32, Float32MultiArray

import apriltag



# constants
LIST_LIGHTS = [miro.constants.LIGHT_LF, miro.constants.LIGHT_RF, miro.constants.LIGHT_LR, miro.constants.LIGHT_RR]
LIST_CLIFFS = [miro.constants.CLIFF_L, miro.constants.CLIFF_R]
LIST_CAMS = [miro.constants.CAM_L, miro.constants.CAM_R]
LIST_MICS = [miro.constants.MIC_L, miro.constants.MIC_R, miro.constants.MIC_C, miro.constants.MIC_T]
LIST_IMUS = [miro.constants.IMU_B, miro.constants.IMU_H]



################################################################
#THIS FUNCTION SHOULD BE DELETED WHEN P2D FUNCTION IS FIXED IN MDK RELEASE
# This function flips the sign of the y axis
def y_correct(y):
	return y * -1

class ClapDetector:

	def __init__(self, cmd_thresh=10, verbose=False):

		# Check threshold is valid value
		self.thresh = np.clip(cmd_thresh, 2, 10)
		if self.thresh != cmd_thresh:
			# Alert user to out of bounds value
			print ("clap theshold is out of bounds, corrected to", self.thresh)

		self.verbose = verbose

		# parameters
		self.background_lambda = 0.1 # IIR filter
		self.sample_time = 0.025	#Blocks recieved at 40hz
		self.max_clap_samples = 0.15/self.sample_time #Max number of samples before considered too long for clap

		# state
		self.clap_mean = [0.0, 0.0, 0.0, 0.0]
		self.background_mean = [0.02, 0.02, 0.02, 0.02] # initialise conservatively
		self.over_threshold_count = [0, 0, 0, 0] #Count for number of samples which exceeded volume threshold
		self.time_at_clap = [None, None, None, None] # Initialise to None
		self.clap = False #Initialise flag for reporting

	def configure(self, cmd_thresh=10, verbose=False):

		# Check threshold is valid value
		self.thresh = np.clip(cmd_thresh, 2, 10)
		if self.thresh != cmd_thresh:
			# Alert user to out of bounds value
			print ("clap theshold is out of bounds, corrected to", self.thresh)

		# Clear previously detected claps
		self.over_threshold_count = [0, 0, 0, 0]
		self.time_at_clap = [None, None, None, None]
		#Alert user to change
		print ("Clap detector threshold changed to", self.thresh)

		if verbose != self.verbose:
			if verbose == True:
				print("Clap detector switched to verbose reporting")
			else:
				print("Clap detector switched to basic reporting")
			self.verbose = verbose

	def get_clap_time(self, channel=None):
		# Check that mic index is valid
		assert channel in LIST_MICS or channel is None,\
			"out-of-range mic channel index passed to detected_clap()"

		# Return time of previous clap
		if channel is None:
			if not any(self.time_at_clap):
				ret = None
			else:
				ret = max(t for t in self.time_at_clap if t is not None)
		else:
			ret = self.time_at_clap[channel]
		return ret


	def calculateRMS(self, sample):

		# Calculate Root Mean Square of data
		count = len(sample)
		data = np.array(sample).astype('float32')
		data *= (1.0 / 32768.0)
		return np.sqrt(np.mean(data**2.0))

	def process(self, mics, time_now):

		rms = [None]*len(LIST_MICS)

		# For each channel
		for channel in LIST_MICS:
			# Calculate RMS for each channel
			rms[channel] = self.calculateRMS(mics.data[:, channel].flatten())

			#Look for samples which exceed threshold
			if rms[channel] > self.thresh * self.background_mean[channel]:
				self.over_threshold_count[channel] += 1 #If sample exceed volume, increment count
				self.clap_mean[channel] += rms[channel]

			#When sample does not exceed threshold, test to see if number of consecutive samples which did is within clap limits
			else:
				# recent clap
				if 1 <= self.over_threshold_count[channel] < self.max_clap_samples:

					# update output
					self.time_at_clap[channel] = time_now
					self.clap_mean[channel] *= (1.0 / self.over_threshold_count[channel])
					if self.verbose:
						level = "{0:.3f}".format(self.clap_mean[channel])
						bg = "{0:.3f}".format(self.background_mean[channel])
						print ("clap @", \
								"{0:.3f}".format(self.time_at_clap[channel]), "sec", \
								", level =", level, ", background =", bg)
					else:
						if channel == 0:
							x = "left"
						elif channel == 1:
							x = "right"
						elif channel == 2:
							x = "head"
						else:
							x = "tail"

						print("Clap detected on " + x + " microphone.")

				# esimate background
				self.background_mean[channel] += self.background_lambda * (rms[channel] - self.background_mean[channel])

				# reset
				self.over_threshold_count[channel] = 0 #Reset count
				self.clap_mean[channel] = 0.0



################################################################
# https://pypi.org/project/apriltag/

class VisionInterface:

	def __init__(self, verbose=True):

		self.verbose = verbose
		self.cam_images = [None, None]
		self.frame_w = None
		self.frame_h = None
		self.frame_area = None
		self.x_cent = None
		self.y_cent = None

		self.topic_root = '/' + os.getenv("MIRO_ROBOT_NAME") + '/'
		self.annotation_pub = rospy.Publisher(self.topic_root + "vision/annotation", miro.msg.img_annotation, queue_size=0)

		april_options = apriltag.DetectorOptions( \
				families='tag16h5',
				border=1,
				nthreads=4,
				quad_decimate=1.0,
				quad_blur=0.0,
				refine_edges=True,
				refine_decode=False,
				refine_pose=False,
				debug=False,
				quad_contours=True)

		self.camera_model_full=None
		self.april_detector = apriltag.Detector(april_options)
		self.tag = miro.msg.object_tag()

		#Generate constants for calculating distance
		#Using formula F = (P*D)/W
		#Where F = "Focal Length", P = Percived Width, D = Distance, W = Width
		#Previous calibration with cube at 0.5m, and Width = 0.057m shows P = 0.0583887183
		#Therefore: F = (0.0583887183 * 0.5)/0.057 = 0.512181739
		self.april_F = 0.512181739
		self.april_FW = 0.057

		#Properties
		self.loc = 0
		self.loc_x = 1
		self.loc_y = 2
		self.cam = 3
		self.dist = 4
		self.rad = 5
		self.area = 6
		self.id = 7
		self.cube_props  = [self.loc, self.loc_x, self.loc_y, self.cam, self.id, self.dist]
		self.ball_props  = [self.loc, self.loc_x, self.loc_y, self.cam, self.rad]
		self.shape_props = [self.loc, self.loc_x, self.loc_y, self.cam, self.area]
		self.com_props = [self.loc, self.loc_x, self.loc_y, self.cam, self.area]

	def process(self, frame):

		if self.camera_model_full is None:
			self.camera_model_full = miro.lib.camera_model.CameraModel(verbose=self.verbose)
			self.camera_model_full.set_frame_size_from_img(frame.data)

		if self.frame_w is None:
			im_h, im_w = frame.data.shape[:2]
			self.frame_w, self.frame_h = im_w, im_h
			self.frame_area = self.frame_w * self.frame_h
			self.x_cent = self.frame_w / 2.0
			self.y_cent = self.frame_h / 2.0

		self.cam_images[frame.camera_index] = frame.data

	def rgb_to_hsv(self, colour):

		#create colour code from user selected colour
		bgr_colour = np.uint8([[[colour[2], colour[1], colour[0]]]])
		hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)
		hue = hsv_colour[0,0][0]
		return hue

	def colour_mask(self, im, hue, hue_range):

		#convert image to hsv colour space
		im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

		# mask image
		hue_min = hue - hue_range
		hue_max = hue + hue_range

		if hue_min < 0:
			lo = np.array([0, 10, 10])
			hi = np.array([hue_max, 255, 255])
			mask1 = cv2.inRange(im_hsv, lo, hi)
			lo = np.array([hue_min + 180, 10, 10])
			hi = np.array([180, 255, 255])
			mask2 = cv2.inRange(im_hsv, lo, hi)
			mask = cv2.bitwise_or(mask1, mask2)
			return mask
		elif hue_max > 180:
			lo = np.array([hue_min, 10, 10])
			hi = np.array([180, 255, 255])
			mask1 = cv2.inRange(im_hsv, lo, hi)
			lo = np.array([0, 10, 10])
			hi = np.array([hue_max-180, 255, 255])
			mask2 = cv2.inRange(im_hsv, lo, hi)
			mask = cv2.bitwise_or(mask1, mask2)
			return mask
		else:
			lo = np.array([hue_min, 10, 10])
			hi = np.array([hue_max, 255, 255])
			mask = cv2.inRange(im_hsv, lo, hi)
			return mask

	def calc_com(self, rgb, index, colour_range=15):

		# get image
		im = self.cam_images[index]
		if im is None:
			return None

		com = [[None, None], None]

		#Convert string to hsv colur value
		hue = self.rgb_to_hsv(rgb)

		#filter image for specified colour
		mask = self.colour_mask(im, hue, colour_range)

		#Count number of matching pxels
		matching_pix = np.sum(mask == 255)
		area_scaled = matching_pix/self.frame_area
		com[1] = area_scaled

		if matching_pix != 0:
			# calculate x,y coordinate of center
			moments = cv2.moments(mask, True)
			cX = int(moments["m10"] / moments["m00"])
			cY = int(moments["m01"] / moments["m00"])
			p = [cX, cY]
			com[0] = self.camera_model_full.p2d(p)
			#Correct Y-Axis from p2d
			com[0][1] = y_correct(com[0][1])

		return com

	def detect_shape(self, rgb, vert, index, colour_range=30, min_size=0.001):

		# get image
		im = self.cam_images[index]
		if im is None:
			return None

		# Initialise object
		shape = [[None, None], None, 0]

		#Convert string to hsv colur value
		hue = self.rgb_to_hsv(rgb)

		#filter image for specified colour
		mask = self.colour_mask(im, hue, colour_range)

		#find contours in image
		# _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		if len(cnts) == 2:
			contours = cnts[0]
		else:
			contours = cnts[1]

		if len(contours) == 0:
			return None
		else:
			# Look for largest contour
			for contour in contours:
				M = cv2.moments(contour)
				if M["m00"] != 0:
					cX = int((M["m10"] / M["m00"]))
					cY = int((M["m01"] / M["m00"]))
					peri = cv2.arcLength(contour, True)
					approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

					# Test for shapes with correct number of vertices
					if len(approx) == vert:
						area = cv2.contourArea(contour)
						area_scaled = area/self.frame_area #Scale area to be proportion of frame
						if area_scaled > shape[2] and area_scaled >= min_size:
							p = [cX, cY]
							shape[0] = self.camera_model_full.p2d(p)
							#Correct Y-Axis from p2d
							shape[0][1] = y_correct(shape[0][1])
							vertices = approx.copy()
							vertices_d = [None] * vert
							for i, coords in enumerate(vertices):
								vertices_d[i] = self.camera_model_full.p2d(coords[0].tolist())
								#Correct Y-Axis from p2d
								vertices_d[i][1] = y_correct(vertices_d[i][1])
							shape[1] = np.asarray(vertices_d)
							shape[2] = area_scaled

			if shape[2] == 0:
				return None
			else:
				return shape

	def detect_april(self, index):

		# get image
		im = self.cam_images[index]
		if im is None:
			return None, None

		#Convert Image to greyscale
		im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

		#Detect Tags
		result = self.april_detector.detect(im)

		#Reset tag distances
		min_distance = None
		distance = None

		if len(result) > 0:
			count = 0
			for i in range(len(result)):
				tag = result[i]

				id = tag[1]
				ham = tag[2]

				# pare by id and hamming distance
				if ham == 0 and id >= 1 and id <= 6:

					count += 1

					# extract fields
					family = tag[0]
					id = tag[1]
					goodness = tag[3]
					decmar = tag[4]
					hom = tag[5]
					cen = tag[6]
					corn = tag[7]

					# convert to d
					cen_d = cen.copy()
					cen_d = self.camera_model_full.p2d(cen_d)
					#Correct Y-Axis from p2d
					cen_d[1] = y_correct(cen_d[1])
					corn_d = corn.copy()
					for i in range(4):
						corn_d[i] = self.camera_model_full.p2d(corn_d[i])
						#Correct Y-Axis from p2d
						corn_d[i][1] = y_correct(corn_d[i][1])

					# flatten
					corn_d = corn_d.flatten()

					c = np.array(corn_d).reshape((4, 2))
					max_length = 0.0
					for i in range(4):
						if i == 3:
							j = 0
						else:
							j = i + 1
						length = np.sqrt(((c[i][0]-c[j][0])**2)+((c[i][1]-c[j][1])**2))
						if length > max_length:
							max_length = length

		            #Calculate DISTANCE
		            #Using formula F = (P*D)/W
		            #Where F = "Focal Length", P = Percived Width, D = Distance, W = Width
		            #Previous calibration with cube at 0.5m, and Width = 0.057m shows P = 0.0583887183
		            #Therefore: F = (0.0583887183 * 0.5)/0.057 = 0.512181739
		            #D = (F*W)/P
		            #F*W is constant as size of cube is constant
					if max_length > 0.0:
						distance = self.april_FW/max_length

					if min_distance is None or distance < min_distance:
						min_distance = distance
						self.tag.conf = goodness
						self.tag.id = id
						self.tag.centre = cen_d
						self.tag.corners = corn_d
						corn_ann = corn.flatten()
						cen_ann = cen

			if min_distance is not None:
				return self.tag, min_distance
			else:
				return None, None

		else:
			return None, None

	def calc_centre_colour(self, size, index):

		# get image
		im = self.cam_images[index]
		if im is None:
			return None

		if size == 1:
			roi_pix = im[int(self.y_cent), int(self.x_cent)]
			blue = roi_pix[0]
			green = roi_pix[1]
			red = roi_pix[2]
		elif size > 1:
			size = np.sqrt(size)
			roi_pix = im[int((self.frame_h - size)/2) : int((self.frame_h + size)/2), int((self.frame_w - size)/2) : int((self.frame_w + size)/2)]
			blue = np.mean(roi_pix[:, :, 0])
			green = np.mean(roi_pix[:, :, 1])
			red = np.mean(roi_pix[:, :, 2])
		else:
			blue = np.mean(im[:, :, 0])
			green = np.mean(im[:, :, 1])
			red = np.mean(im[:, :, 2])

		colour = [int(red), int(green), int(blue)]
		return colour

	def detect_ball(self, rgb, index, colour_range=15, min_rad_scaled=0.0078125):

		# get image
		im = self.cam_images[index]
		if im is None:
			return None

		#create colour code from user selected colour
		hue = self.rgb_to_hsv(rgb)

		#filter image for specified colour
		mask = self.colour_mask(im, hue, colour_range)

		# clean up - remove noise
		seg = mask
		seg = cv2.GaussianBlur(seg, (5, 5), 0)
		seg = cv2.erode(seg, None, iterations=2)
		seg = cv2.dilate(seg, None, iterations=2)

		# parameters
		canny_high_thresh = 128 # doesn't matter much for binary image
		ball_detect_sensitivity = 15 # lower detects more circles, so it's a trade-off
		ball_detect_min_dist_between_cens = 40 # Tuned to work well
		ball_detect_min_radius = int(min_rad_scaled * self.frame_w) # default is 5. Too small and we'll pick up noise objects
		ball_detect_max_radius = 60 # too small and we'll miss valid circles, too large and we'll pick up spurious results

		# get circles
		circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT,
				1, ball_detect_min_dist_between_cens, \
				param1=canny_high_thresh, param2=ball_detect_sensitivity, \
				minRadius=ball_detect_min_radius, maxRadius=0)

		# Get largest circle
		max_circle = None
		if circles is not None:
			max_rad = 0
			circles = np.uint16(np.around(circles))

			for c in circles[0,:]:
				if c[2] > max_rad:
					max_rad = c[2]
					max_circle = c

			if not max_circle is None:
				max_circle = np.array(max_circle).astype('float32')
				p = [max_circle[0], max_circle[1]]
				cen_d = self.camera_model_full.p2d(p)
				#Correct Y-Axis from p2d
				cen_d[1] = y_correct(cen_d[1])
				rad = max_circle[2]/self.frame_w # scale radius to be as a proportion of image width

				return [cen_d, rad]

			return None

		else:
			return None



################################################################

class MirocodeInterface:

	def __init__(self, pose_ctrl=True, cliff_reflex=True, robot_name=None, control_period=0.1):

		flags = 0
		if not cliff_reflex:
			flags |= miro.constants.PLATFORM_D_FLAG_DISABLE_CLIFF_REFLEX

		self.interface = miro.lib.RobotInterface(
				None,
				robot_name=robot_name,
				control_period=control_period,
				flags=flags,
				command='frame=360w@5',
				mirocode_mode=True,
				use_pose_control=pose_ctrl
				)

		# Interface MiroCode state
		self.active = True
		self.sensor_period = 0.02

		# Create audio and vision objects for processing these inputs
		self.vision = VisionInterface(False)
		self.clap_detector = ClapDetector()

		## STATE ##

		# flags
		self.sensors_updated = False

		# Construct objects for velocity corrections
		self.cmd_vel = miro.lib.DeltaPose()
		self.cmd_lin_vel = 0.0
		self.cmd_ang_vel = 0.0

		#  Wag state
		self.wag_freq = 0.0

		# Timer Variables
		self.time_limit = 0
		self.timer_watchdog_t = 0
		self.timer_watchdog_n = 0

		# Touch state
		long_ago = -300.0
		self.time_at_touch = long_ago
		self.time_at_head_touch = long_ago
		self.time_at_body_touch = long_ago

		# Clap State
		self.current_clap = [False, False, False, False]
		self.time_at_prev_clap = 0.0

		# Shake calculations
		self.shake_calc_in_prog = False
		self.shake_smooth_i = [0, 0]
		self.shake_smooth = [[0.0] * 3] * 2
		self.shake = [None, None]
		self.time_at_shake = [None, None]
		self.shake_detected = [False, False]
		self.shake_thresh = 2.0
		self.count = 0
		self.new_accel_data = [[None]*3] *2
		self.old_accel_data = [[None]*3] *2

		# Register for callbacks
		self.interface.register_callback("sensors", self.sensors_callback)
		self.interface.register_callback("camera_left", self.camera_l_callback)
		self.interface.register_callback("camera_right", self.camera_r_callback)
		self.interface.register_callback("microphones", self.microphones_callback)

		# Create  ROS publisher for annotating images in MiRoCode
		topic_base_name = "/" + self.interface.robot_name
		self.pub_annotation = rospy.Publisher(topic_base_name + "/vision/annotation", miro.msg.img_annotation, queue_size=0)

		# create control thread
		self.mirocode_thread_active = True
		self.thread = threading.Thread(target = self.mirocode_update)
		self.thread.start()

	def target_mdk_release(self):
		return self.interface.target_mdk_release()

	def target_supports(self, feature):
		return self.interface.target_supports(feature)

	def ready(self):

		if self.is_active():

			# Wait for new data to arrive
			while not self.sensors_updated:
				time.sleep(self.sensor_period * 0.25)

			# Clear Flag
			self.sensors_updated = False

			return True

		else:

			return False

	def sensors_callback(self, sensors):

		# Store old accel data
		self.old_accel_data[miro.constants.IMU_H] = list(self.new_accel_data[miro.constants.IMU_H])
		self.old_accel_data[miro.constants.IMU_B] = list(self.new_accel_data[miro.constants.IMU_B])

		# Store new accel data
		self.new_accel_data[miro.constants.IMU_H] = list(sensors.accel_head)
		self.new_accel_data[miro.constants.IMU_B] = list(sensors.accel_body)

		if any(sensors.touch_head):
			self.time_at_touch = self.interface.get_time()
			self.time_at_head_touch = self.interface.get_time()

		if any(sensors.touch_body):
			self.time_at_touch = self.interface.get_time()
			self.time_at_body_touch = self.interface.get_time()

		# Run shake detector
		if not self.shake_calc_in_prog:
			self.calculate_shake()

		# Set flag to show data has arrived
		self.sensors_updated = True

	def camera_l_callback(self, frame):
		self.vision.process(frame)

	def camera_r_callback(self, frame):
		self.vision.process(frame)

	def microphones_callback(self, microphones):
		self.clap_detector.process(microphones, self.interface.get_time())

	def is_active(self):
		return self.active and self.interface.is_active()

	def mirocode_update(self):

		try:

			# until term() called or ROS falls over
			while self.is_active():

				# get time and wait for control boundary
				t = self.interface.wait_for_control_boundary()

				# Wag tail if frequency is not 0
				if self.wag_freq > 0.0:
					phase = np.pi * 2 * self.wag_freq * t
					wag_pos = 0.5 + 0.5 * np.sin(phase)
					self.joint_position(miro.constants.JOINT_WAG, wag_pos)

				# Command speeds
				self.interface.set_vel(lin_vel=self.cmd_lin_vel, ang_vel=self.cmd_ang_vel)

				# Test if clap was detected in previous quarter of a second (10 audio packets)
				for channel in LIST_MICS:

					# default false
					self.current_clap[channel] = False

					# condition true
					if self.clap_detector.time_at_clap[channel] is not None:
						if self.clap_detector.time_at_clap[channel] >= (t - 0.25) and self.time_at_prev_clap < (t - 0.25):
							self.current_clap[channel] = True
							self.time_at_prev_clap = t

		except Exception as e:

			logging.error(traceback.format_exc())

		# mark thread is exiting
		self.mirocode_thread_active = False

	def term(self):

		self.active = False

	def disconnect(self):

		#Close the Mirocode Interface
		self.term()
		#sys.stdout.write("wait for mirocode thread terminate... ")
		#sys.stdout.flush()
		while self.mirocode_thread_active:
			time.sleep(0.1)
		#print("OK")

		# Close the interface
		self.interface.disconnect()

	#### CONTROL ###########################################################

	def speed(self, m_sec):
		self.cmd_lin_vel = m_sec
		# self.interface.set_vel(lin_vel=m_sec)

	def turn_speed(self, deg_sec):
		# convert from degrees to radians
		rad_sec = math.radians(deg_sec)
		self.cmd_ang_vel = rad_sec
		# self.interface.set_vel(ang_vel=rad_sec)

	def drive_distance(self, m, m_sec=0.3):

		# Speed should always be positive because
		# direction is controlled by distance
		speed = abs(m_sec)

		# Set direction
		if m > 0.0:
			vel = speed
		else:
			vel = -speed

		# Get current location relative to start pose
		pose_est = self.interface.get_pose()
		x = pose_est.x
		y = pose_est.y
		theta = pose_est.theta

		# Get target location relative to start pose
		target_x = x + np.cos(theta)*m
		target_y = y + np.sin(theta)*m

		# Calculate change in x and y required
		if target_x > 0:
			dx = target_x - x
		else:
			dx = x - target_x
		if target_y > 0:
			dy = target_y - y
		else:
			dy = y - target_y

		# Control to nearly 0 to avoid overshoot and error
		while dx > 0.1 or dy > 0.1:

			time.sleep(self.sensor_period)
			self.speed(vel)

			# Recalculate change in x and y required
			#pose_est = self.interface.get_pose()
			if target_x > 0:
				dx = target_x - pose_est.x
			else:
				dx = pose_est.x - target_x
			if target_y > 0:
				dy = target_y - pose_est.y
			else:
				dy = pose_est.y - target_y

		# Stop Driving
		self.speed(0.0)

	def turn_angle(self, deg, deg_sec=30):

		# Speed should always be positive because
		# direction is controlled by angle
		speed = abs(deg_sec)

		# Convert degrees to radians
		rad = math.radians(deg)

		# Set direction
		if rad < 0.0:
			clockwise = True
		else:
			clockwise = False

		#Get current Angle from Pose Controller
		pose_est = self.interface.get_pose()
		start_ang = pose_est.theta

		#Calculate Target Angle
		target_ang = start_ang + rad
		if clockwise:
			d_ang = pose_est.theta - target_ang
			vel = -speed
		else:
			d_ang = target_ang - pose_est.theta
			vel = speed

		# Turn until nearly 0 to prevent overshoot
		while d_ang > 0.1:
			time.sleep(self.sensor_period)
			self.turn_speed(vel)
			if clockwise:
				d_ang = pose_est.theta - target_ang
			else:
				d_ang = target_ang - pose_est.theta

		# Stop turning
		self.turn_speed(0)

	def neck_angle(self, joint, deg):
		# convert from degrees to radians
		rad = math.radians(deg)
		self.interface.set_kin(joint, rad)

	def joint_position(self, joint, pos):
		self.interface.set_cos(joint, pos)

	def wag_frequency(self, hz):
		# Check value is within bounds
		self.wag_freq = self.interface.clip_with_warn(hz, -2, 2, "wag_frequency", "frequency")

	def control_led(self, led, rgb, brightness):
		self.interface.set_illum(led, rgb, brightness)

	def play_tone(self, hz, sec, vol):
		# Convert duration in seconds to number of samples at 50Hz
		samp = int(sec*50)
		self.interface.post_tone(hz, samp, vol)

	#### READS #############################################################

	def read_cliff_sensor(self, pos, thresh=0.7, return_values=False):
		# check cliff sensor index is valid
		assert pos in LIST_CLIFFS

		if return_values:
			return self.interface.get_cliff()[pos]
		else:
			# Convert to bool based on threshold value and return
			return self.interface.get_cliff()[pos] < thresh

	def read_cliff_sensor_list(self, thresh=0.7, return_values=False):

		# Create list to hold cliff values
		cliff_list = [None, None]

		if return_values:
			for i, cliff in enumerate(self.interface.get_cliff()):
				cliff_list[i] = cliff
		else:
			# Convert cliff values to bools based on threshold value
			for i, cliff in enumerate(self.interface.get_cliff()):
				cliff_list[i] = cliff < thresh

		return cliff_list

	def read_sonar_range(self):
		return self.interface.get_sonar()

	def read_light_sensor(self, pos):
		# check light sensor index is valid
		assert pos in LIST_LIGHTS
		# Return requested value
		return self.interface.get_light()[pos]

	def read_light_sensor_list(self):
		return self.interface.get_light()

	def read_head_touch_sensor_list(self):
		return self.interface.get_touch_head()

	def read_body_touch_sensor_list(self):
		return self.interface.get_touch_body()

	def read_head_acceleration(self):
		return self.interface.get_accel_head()

	def read_body_acceleration(self):
		return self.interface.get_accel_body()

	#### CALCULATE #########################################################

	def calculate_shake(self):

		self.shake_calc_in_prog = True

		# For each IMU
		self.count += 1
		for imu in LIST_IMUS:

			# If stored value already exists
			if self.old_accel_data[0][0] is not None:
				d_acc = [0.0, 0.0, 0.0]
				# Calculate magnitude
				for i in range(3):
					d_acc[i] = self.new_accel_data[imu][i] - self.old_accel_data[imu][i]
				shake_temp = np.sqrt(np.square(d_acc[0]) + np.square(d_acc[1]) + np.square(d_acc[2]))

				#Smooth magnitudes using 2 previous values
				if self.shake_smooth_i[imu] == 3:
					self.shake_smooth_i[imu] = 0
				self.shake_smooth[imu][self.shake_smooth_i[imu]] = shake_temp
				self.shake[imu] = np.mean(self.shake_smooth[imu])
				self.shake_smooth_i[imu] += 1

				# Test shake magnitudes against threshold
				if self.shake[imu] > self.shake_thresh:
					self.shake_detected[imu] = True
					self.time_at_shake[imu] = self.interface.get_time()

		self.shake_calc_in_prog = False

	def configure_shake_detector(self, thresh=10):

		# Set new threshold
		self.shake_thresh = thresh
		#clear old shakes
		self.shake_detected = [False, False]
		self.time_at_shake = [None, None]
		# alert user to change
		print ("Shake detector threshold changed to", self.shake_thresh)

	def detect_shake(self, imu=None):

		# Check imu index is valid
		assert imu in LIST_IMUS or imu is None,\
			"out-of-range imu index passed to detect_shake()"

		# if any value is not none
		if imu is None:
			ret = any(self.shake_detected)
			self.shake_detected = [False, False]
		else:
			ret = self.shake_detected[imu]
			self.shake_detected[imu] = False

		return ret

	def time_since_shake(self, imu=None):

		# Check imu index is valid
		assert imu in LIST_IMUS or imu is None,\
			"out-of-range imu index passed to time_since_shake()"

		# Get time now
		time_now = self.interface.get_time()
		shake_time = None

		# Get time of shake
		if imu is None:
			shake_time = max(self.time_at_shake)
		else:
			shake_time = self.time_at_shake[imu]

		if shake_time is None:
			return None
		else:
			return time_now - shake_time

	def time_since_touch(self, pos="any"):

		assert pos in ["head", "body", "any"],\
			"invalid body part passed to time_since_touch()"

		if pos == "head":
			return self.interface.get_time() - self.time_at_head_touch
		elif pos == "body":
			return self.interface.get_time() - self.time_at_body_touch
		else:
			return self.interface.get_time() - self.time_at_touch

	#### AUDIO #############################################################

	def detect_clap(self, mic=None):
		# Check mic index is valid
		assert mic in LIST_MICS or mic is None,\
			"out-of-range mic index passed to detect_clap()"

		if mic is None:
			ret = any(self.current_clap)
			self.current_clap = [False, False, False, False]
		else:
			ret = self.current_clap[mic]
			self.current_clap[mic] = False

		return ret

	def configure_clap_detector(self, thresh):
		self.clap_detector.configure(thresh)

	def time_since_clap(self, mic=None):
		# Check mic index is valid
		assert mic in LIST_MICS or mic is None,\
			"out-of-range mic index passed to time_since_clap()"

		time_now = self.interface.get_time()
		clap_time = self.clap_detector.get_clap_time(mic)
		if clap_time is None:
			return None
		else:
			return time_now - clap_time

	#### VISION ############################################################

	def find_mirocube(self, cam=None, prop=None):
		assert cam in LIST_CAMS or cam is None,\
			"out-of-range camera index passed to find_mirocube"

		assert prop in self.vision.cube_props or prop == None,\
			"invalid property passed to find_mirocube"

		if cam is not None:
			mirocube, dist = self.vision.detect_april(cam)
			index = cam
		else:
			# Find april tags in bothe images
			left_april, l_dist = self.vision.detect_april(0)
			right_april, r_dist = self.vision.detect_april(1)

			# If none found return Nones
			if left_april is None and right_april is None:
				return None

			# If tag in right image
			elif left_april is None and right_april is not None:
				mirocube = right_april
				dist = right_dist
				index = miro.constants.CAM_R

			# If tag in left image
			elif left_april is not None and right_april is None:
				mirocube = left_april
				dist = left_dist
				index = miro.constants.CAM_L

			# If in both, find the closest
			else:
				if r_dist > l_dist:
					mirocube = right_april
					dist = right_dist
					index = miro.constants.CAM_R
				else:
					mirocube = left_april
					dist = left_dist
					index = miro.constants.CAM_L

		annotate = miro.msg.img_annotation()
		if mirocube is None:
			annotate.type.data = "clear"
		else:
			annotate.type.data = "apriltag"
			if index == 0:
				annotate.cam.data = "left"
			else:
				annotate.cam.data = "right"
			annotate.id.data = mirocube.id
			annotate.centre.data = mirocube.centre
			annotate.vertices.data = mirocube.corners
			annotate.distance.data = dist

		self.pub_annotation.publish(annotate)

		if mirocube is None:
			return None

		if prop == self.vision.id:
			return mirocube.id
		elif prop == self.vision.loc:
			return mirocube.centre
		elif prop == self.vision.loc_x:
			return mirocube.centre[0]
		elif prop == self.vision.loc_y:
			return mirocube.centre[1]
		elif prop == self.vision.dist:
			return dist
		elif prop == self.vision.cam:
			return index
		else:
			return mirocube, index

	def find_colour(self, rgb, cam=None, prop=None, colour_range=15):
		assert cam in LIST_CAMS or cam is None,\
			"out-of-range camera index passed to find_colour"

		assert prop in self.vision.com_props or prop == None,\
			"invalid property passed to find_colour"

		if cam is not None:
			colour = self.vision.calc_com(rgb, cam, colour_range)
			index = cam
		else:
			colour_l = self.vision.calc_com(rgb, miro.constants.CAM_L, colour_range)
			colour_r = self.vision.calc_com(rgb, miro.constants.CAM_R, colour_range)

			if colour_l is None and colour_r is None:
				colour = None
			elif colour_l is None and colour_r is not None:
				colour = colour_r
				index = miro.constants.CAM_R
			elif colour_r is None and colour_l is not None:
				colour = colour_l
				index = miro.constants.CAM_L
			else:
				if colour_l[1] > colour_r[1]:
					colour = colour_l
					index = miro.constants.CAM_L
				else:
					colour = colour_r
					index = miro.constants.CAM_R
		if colour is None:
			return None

		annotate = miro.msg.img_annotation()

		#If no pixels detected do not annotate
		if colour[1] == 0:
			annotate.type.data = "clear"
		else:
			annotate.type.data = "colour"
			if index == miro.constants.CAM_L:
				annotate.cam.data = "left"
			else:
				annotate.cam.data = "right"
			annotate.centre.data = colour[0]
			annotate.area.data = colour[1]

		self.pub_annotation.publish(annotate)

		if prop == self.vision.loc:
			return colour[0]
		elif prop == self.vision.loc_x:
			return colour[0][0]
		elif prop == self.vision.loc_y:
			return colour[0][1]
		elif prop == self.vision.area:
			return colour[1]
		elif prop == self.vision.cam:
			return index
		else:
			return colour, index

	def find_shape(self, rgb, verts, cam=None, prop=None, colour_range=30, min_size=0.001):
		assert cam in LIST_CAMS or cam is None,\
			"out-of-range camera index passed to find_shape"

		assert 3 <= verts <= 8,\
			"invalid number of vertices (3-8) passed to find_shape"

		assert prop in self.vision.shape_props or prop == None,\
			"invalid property passed to find_shape"

		if cam is None:
			shape_l = self.vision.detect_shape(rgb, verts, miro.constants.CAM_L, colour_range, min_size)
			shape_r = self.vision.detect_shape(rgb, verts, miro.constants.CAM_R, colour_range, min_size)

			if shape_l is None and shape_r is None:
				shape = None
			elif shape_l is None and shape_r is not None:
				shape = shape_r
				index = miro.constants.CAM_R
			elif shape_r is None and shape_l is not None:
				shape = shape_l
				index = miro.constants.CAM_L
			else:
				if shape_l[1] > shape_r[1]:
					shape = shape_l
					index = miro.constants.CAM_L
				else:
					shape = shape_r
					index = miro.constants.CAM_R

		else:
			shape = self.vision.detect_shape(rgb, verts, cam, colour_range, min_size)
			index = cam

		annotate = miro.msg.img_annotation()

		if shape is None:
			annotate.type.data = "clear"
		else:
			annotate.type.data = "shape"
			if index == 0:
				annotate.cam.data = "left"
			else:
				annotate.cam.data = "right"
			annotate.centre.data = shape[0]
			vert_array =shape[1].flatten()
			vert_list = vert_array.tolist()
			annotate.vertices.data = vert_list
			annotate.area.data = shape[2]
			annotate.size.data = verts

		self.pub_annotation.publish(annotate)

		if shape is None:
			return None

		if prop == self.vision.loc:
			return shape[0]
		elif prop == self.vision.loc_x:
			return shape[0][0]
		elif prop == self.vision.loc_y:
			return shape[0][1]
		elif prop == self.vision.area:
			return shape[2]
		elif prop == self.vision.cam:
			return index
		else:
			return shape, index

	def find_ball(self, rgb, cam=None, prop="None", colour_range=15, min_rad=0.0078125):
		assert cam in LIST_CAMS or cam is None,\
			"out-of-range camera index passed to find_mirocube_property"

		assert prop in self.vision.ball_props or prop == None,\
			"invalid property passed to find_ball"

		if cam is None:
			ball_l = self.vision.detect_ball(rgb, miro.constants.CAM_L, colour_range, min_rad)
			ball_r = self.vision.detect_ball(rgb, miro.constants.CAM_R, colour_range, min_rad)

			if ball_l is None and ball_r is None:
				ball = None
			elif ball_l is None and ball_r is not None:
				ball = ball_r
				index = miro.constants.CAM_R
			elif ball_r is None and ball_l is not None:
				ball = ball_l
				index = miro.constants.CAM_L
			else:
				if ball_l[1] > ball_r[1]:
					ball = ball_l
					index = miro.constants.CAM_L
				else:
					ball = ball_r
					index = miro.constants.CAM_R

		else:
			ball = self.vision.detect_ball(rgb, cam, colour_range, min_rad)
			index = cam

		annotate = miro.msg.img_annotation()

		if ball is None:
			annotate.type.data = "clear"
		else:
			annotate.type.data = "ball"
			if index == 0:
				annotate.cam.data = "left"
			else:
				annotate.cam.data = "right"
			annotate.centre.data = ball[0]
			annotate.size.data  = ball[1]

		self.pub_annotation.publish(annotate)

		if ball is None:
			return None

		if prop == self.vision.loc:
			return ball[0]
		elif prop == self.vision.loc_x:
			return ball[0][0]
		elif prop == self.vision.loc_y:
			return ball[0][1]
		elif prop == self.vision.rad:
			return ball[1]
		elif prop == self.vision.cam:
			return index
		else:
			return ball, index

	def find_colour_at_centre(self, size, cam):

		assert cam in LIST_CAMS,\
			"out-of-range camera index passed to find_colour_at_centre"

		colour = self.vision.calc_centre_colour(size, cam)
		return colour

	#### MISC ###################################################

	def sleep(self, dur):

		t0 = self.interface.get_time() #Get current time
		t1 = t0 + dur #Set sleep end time
		watchdog = 0

		while True:

			# If time limit reached, exit
			t = self.interface.get_time()
			if t >= t1:
				break

			# Check to see if timer is increasing
			if t == t0:
				watchdog += 1
				if watchdog == 50:
					raise Exception("timeout in sleep()")
			else:
				watchdog = 0
			t0 = t

			# sleep
			time.sleep(self.sensor_period)

	def set_time_limit(self, dur):
		self.time_limit = self.interface.get_time() + dur

	def wait_for_time_limit(self):
		# Get current time
		t = self.interface.get_time()

		# Watchdog
		if t == self.timer_watchdog_t:
			self.timer_watchdog_n += 1
			if self.timer_watchdog_n == 100:
				raise Exception("timeout in wait_for_time_limit()")
		self.timer_watchdog_n = 0
		self.timer_watchdog_t = t

		# If time limit not reached
		if t < self.time_limit:
			return True

		# If time limit reached
		self.time_limit = 0
		return False
