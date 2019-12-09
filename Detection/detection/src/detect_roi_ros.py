#!/usr/bin/env python

#from __future__ import (absolute_import, division, print_function, unicode_literals)
import cv2 
import numpy as np 
from imutils import contours
from skimage import measure
import numpy as np
import argparse
import imutils
import cv2

import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Point
from detection.msg import point_list

def kalman_xy(x, P, measurement, R,
              motion = np.matrix('0. 0. 0. 0.').T,
              Q = np.matrix(np.eye(4))):
    """
    Parameters:    
    x: initial state 4-tuple of location and velocity: (x0, x1, x0_dot, x1_dot)
    P: initial uncertainty convariance matrix
    measurement: observed position
    R: measurement noise 
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    """
    return kalman(x, P, measurement, R, motion, Q,
                  F = np.matrix('''
                      1. 0. 1. 0.;
                      0. 1. 0. 1.;
                      0. 0. 1. 0.;
                      0. 0. 0. 1.
                      '''),
                  H = np.matrix('''
                      1. 0. 0. 0.;
                      0. 1. 0. 0.'''))

def kalman(x, P, measurement, R, motion, Q, F, H):
    '''
    Parameters:
    x: initial state
    P: initial uncertainty convariance matrix
    measurement: observed position (same shape as H*x)
    R: measurement noise (same shape as H)
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    F: next state function: x_prime = F*x
    H: measurement function: position = H*x

    Return: the updated and predicted new values for (x, P)

    See also http://en.wikipedia.org/wiki/Kalman_filter

    This version of kalman can be applied to many different situations by
    appropriately defining F and H 
    '''
    # UPDATE x, P based on measurement m    
    # distance between measured and current position-belief
    y = np.matrix(measurement).T - H * x
    S = H * P * H.T + R  # residual convariance
    K = P * H.T * S.I    # Kalman gain
    x = x + K*y
    I = np.matrix(np.eye(F.shape[0])) # identity matrix
    P = (I - K*H)*P

    # PREDICT x, P based on motion
    x = F*x + motion
    P = F*P*F.T + Q

    return x, P


def point_publisher():
	
	################## CHANGE VIDEO FOR CAMERA Stream
	cap = cv2.VideoCapture("/home/debjoy/A_projects/interiit/vids_n_pics/cropped_new1.avi")

	# Check if camera opened successfully
	if (cap.isOpened() == False): 
		print("Unable to read feed")

	# cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
	pub = rospy.Publisher('roi_coordinates', point_list)
	rospy.init_node('detect_roi_ros', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	msg = point_list()

	list_prevkalman = []
	list_currentkalman = []

	while not rospy.is_shutdown():
		
		list_currentkalman = []
		ret,frame = cap.read()
		if not ret:
			print "Finished"
			break

		cntlist, list_prevkalman, list_currentkalman = main_func(frame, list_prevkalman, list_currentkalman)
		cv2.waitKey(1)
		if not cntlist:
			rate.sleep()
			continue
		p=0       # Keeping track of cnts published
		#[(x, y, 0), (x+w, y, 0), (x, y+h, 0), (x+h, y+h, 0)]
		for cnts in cntlist:
			p+=1
			#print(p)
			[x, y, w, h] = cnts
			point1 = Point()
			point1.x = x
			point1.y = y
			point1.z = 0
			point2 = Point()
			point2.x = x+w
			point2.y = y
			point2.z = 0
			point3 = Point()
			point3.x = x
			point3.y = y+h
			point3.z = 0
			point4 = Point()
			point4.x = x+w
			point4.y = y+h
			point4.z = 0
			msg.points = [point1, point2, point3, point4]

			rospy.loginfo(msg)
			pub.publish(msg)
			rate.sleep()

def avg_col(img):
	rows = img.shape[0]
	cols = img.shape[1]
	colour = np.int32([np.sum(img[:, :, 0])/(rows*cols), np.sum(img[:, :, 1])/(rows*cols), np.sum(img[:, :, 2])/(rows*cols)])
	return colour 

def inbtwn(color):
	if color[0]>=39 and color[0]<=100 and color[1]>=105 and color[2]>=40:
		return True
	return False

def main_func(frame, list_prevkalman, list_currentkalman):
	
	#key = cv2.waitKey(1) & 0xFF

	#if key == ord('a'):
	cntlist = []

	if True:

		image = frame[:1000, :, :]
		#image = cv2.resize(image, (image.shape[1]//2, image.shape[0]//2))
		cv2.imshow("frame",image)

		gray = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		blur = (cv2.GaussianBlur(gray, (11, 11), 0))
		blurred = blur[:, :, 1]
		cv2.imshow("blurgray", blurred)

		# threshold the image to reveal light regions in the
		# blurred image
		thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)[1]
		# cv2.imshow("smothri", thresh)

		# perform a series of erosions and dilations to remove
		# any small blobs of noise from the thresholded image
		thresh = cv2.erode(thresh, None, iterations=2)
		thresh = cv2.dilate(thresh, None, iterations=4)
		cv2.imshow("theresh", thresh)

		# perform a connected component analysis on the thresholded
		# image, then initialize a mask to store only the "large"
		# components
		labels = measure.label(thresh, neighbors=8, background=0)
		mask = np.zeros(thresh.shape, dtype="uint8")
		# loop over the unique components
		for label in np.unique(labels):
			# if this is the background label, ignore it
			if label == 0:
				continue

			# otherwise, construct the label mask and count the
			# number of pixels 
			labelMask = np.zeros(thresh.shape, dtype="uint8")
			labelMask[labels == label] = 255
			numPixels = cv2.countNonZero(labelMask)

			# if the number of pixels in the component is sufficiently
			# large, then add it to our mask of "large blobs"
			if numPixels > 100:
				mask = cv2.add(mask, labelMask)

		# find the contours in the mask, then sort them from left to
		# right
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		if len(cnts):
			cnts = contours.sort_contours(cnts)[0]

		# loop over the contours
		for (i, c) in enumerate(cnts):
			# draw the bright spot on the image
			(x, y, w, h) = cv2.boundingRect(c)

			cntimg = image[y:y+h, x:x+w, :]
			cnt_extnd_wnd = image[max(0, y-15):min(y+h+15, image.shape[0]), max(0, x-15):min(x+w+15, image.shape[1]), :]

			color_hsv = avg_col(cv2.cvtColor(cntimg, cv2.COLOR_BGR2HSV))
			# print("colour "+": "+str(color_hsv))

			# HSV color threshold check
			if not inbtwn(color_hsv):
				#print("False Positive removed")
				continue

			#####################################
			# Window colour difference check
			color_img = avg_col(cntimg)
			color_extnd_wnd = avg_col(cnt_extnd_wnd)
			# print("cntwind: "+str(color_extnd_wnd))
			# print("cnt: "+str(color_img))
			diffs = (np.mean(np.abs(color_extnd_wnd-cnt_extnd_wnd)))
			if diffs<5:
				#print("False positive removed")
				continue
			#####################################


			#####################################
			# Feature density check
			
			#####################################
			
			# cntlist.append([x,y,w,h])
			((cX, cY), radius) = cv2.minEnclosingCircle(c)
			# cv2.circle(image, (int(cX), int(cY)), int(radius), (255, 255, 255), 3)
			# cv2.putText(image, "#{}".format(i + 1), (x, y - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

			cl = np.array([cX, cY])
			matched = False
			R = 0

			for (xx, P), dc in list_prevkalman:
				# print("inside")
				lcv = P[:2, :2]
				t = np.array([cX-xx[0][0], cY - xx[1][0]]).reshape((1,2))
				lpp = np.matmul(np.matmul(t, lcv), t.T)
				# print("\n\n")
				# print(lpp)
				if lpp < 50000:
					matched = True
					list_currentkalman.append((kalman_xy(xx, P, cl, R), dc+1))
					if dc>=5:
						cv2.circle(image, (int(cX), int(cY)), int(radius),	(0, 0, 255), 3)
						cntlist.append(cv2.boundingRect(c))

			if not matched:
				xx = np.matrix('0. 0. 0. 0.').T 
				P = np.matrix(np.eye(4))*10 # initial uncertainty
				list_currentkalman.append((kalman_xy(xx, P, cl, R), 0))
			
		for (xx, P), dc in list_prevkalman:
			if dc > 5:
				cv2.circle(image, (int(xx[0][0]), int(xx[1][0])), 10, (255, 0, 0), 3)

		list_prevkalman = list_currentkalman
		new_image = cv2.resize(image, (image.shape[1]//2, image.shape[0]//2))
		cv2.imshow("Image", new_image)
		# show the output image
		return cntlist, list_prevkalman, list_currentkalman

point_publisher()