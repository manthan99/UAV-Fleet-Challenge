#!/usr/bin/env python
import sys
print(sys.path)

#from __future__ import (absolute_import, division, print_function, unicode_literals)
import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Point
from detection.msg import point_list

# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")

import cv2 
import numpy as np 

import numpy as np
import argparse

import cv2
font = cv2.FONT_HERSHEY_COMPLEX


# from cv_bridge import CvBridge, CvBridgeError

import tello_frame_detection as jkla
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
	cap = cv2.VideoCapture("/home/archit/Documents/INter IIT/2018_0106_201506_004.MOV")

	# Check if camera opened successfully
	if (cap.isOpened() == False): 
		print("Unable to read feed")

	# cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
	pub = rospy.Publisher('roi_coordinates', point_list)
	# image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

	# bridge = CvBridge()
 
	rospy.init_node('detect_roi_ros', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	msg = point_list()

	list_currentdetect = []
	list_prevkalman = []
	list_currentkalman = []
	list_matched = []
	ind = 0
	while not rospy.is_shutdown():
		ind +=1
		print(ind)
		ret,frameorg = cap.read()
		frameorg = cv2.resize(frameorg, (frameorg.shape[1]//2, frameorg.shape[0]//2))
		cv2.waitKey(1)
		if ret == True:
			if ind<5500:
				continue
			frame = frameorg
			gray = cv2.cvtColor(frameorg, cv2.COLOR_BGR2GRAY)
			corners = cv2.goodFeaturesToTrack(gray,0,0.0001,0.01)
			ctc = np.zeros(gray.shape)
			list_currentkalman = []
			list_matched = []
			for i in corners:
				x,y = i.ravel()
				ctc[int(y)][int(x)]=1
				#cv2.circle(frameorg,(x,y),1,255,-1)



			t = (jkla.get_cnt(frame))

			frame = cv2.GaussianBlur(frame, (11,11), 0)
			res2 = t[2]
			# print(t[1])
			cnt_images = []
			contourlist = []
			for cnt in t[1]:
				rects_temp = cv2.boundingRect(cnt)

				rects = np.array([[rects_temp[0], rects_temp[1]], [rects_temp[0]+rects_temp[2], rects_temp[1]], [rects_temp[0]+rects_temp[2], rects_temp[1]+rects_temp[3]], [rects_temp[0], rects_temp[1]+rects_temp[3]]])
				rects = rects.reshape(cnt.shape)
				center =  np.sum(rects, axis=0)/4
				rects = np.maximum((0.5*(rects - center) + center).astype(int), np.zeros(rects.shape)).astype(int)
				rects_temp = cv2.boundingRect(rects)

				rectl = np.maximum((3*(rects - center) + center).astype(int), np.zeros(rects.shape)).astype(int)
				rectl_temp = cv2.boundingRect(rectl)
				rectl_temp = [rectl_temp[0], rectl_temp[1], rectl_temp[2], rectl_temp[3]]
				if rectl_temp[0]+rectl_temp[2] >= ctc.shape[1]:
					rectl_temp[2] = ctc.shape[1]-rectl_temp[0]-1
				if rectl_temp[1]+rectl_temp[3] >= ctc.shape[0]:
					rectl_temp[3] = ctc.shape[0]-rectl_temp[1]-1
				sel = np.ix_(np.arange(rects_temp[1], rects_temp[1]+rects_temp[3]).tolist(), np.arange(rects_temp[0], rects_temp[0]+rects_temp[2]).tolist())
				selc = np.ix_(np.arange(rects_temp[1], rects_temp[1]+rects_temp[3]).tolist(), np.arange(rects_temp[0], rects_temp[0]+rects_temp[2]).tolist(), [0,1,2])
				acs = np.sum(frameorg[selc], axis=(0,1)).astype(np.float)/rects_temp[2]/rects_temp[3]
				fcs = np.sum(ctc[(sel)])
				vs = np.var(frame[selc])
				sel = np.ix_(np.arange(rectl_temp[1], rectl_temp[1]+rectl_temp[3]).tolist(), np.arange(rectl_temp[0], rectl_temp[0]+rectl_temp[2]).tolist())
				selc = np.ix_(np.arange(rectl_temp[1], rectl_temp[1]+rectl_temp[3]).tolist(), np.arange(rectl_temp[0], rectl_temp[0]+rectl_temp[2]).tolist(), [0,1,2])
				fcl = np.sum(ctc[(sel)])
				acl = np.sum(frameorg[selc], axis=(0,1)).astype(np.float)/rectl_temp[2]/rectl_temp[3]
				
				d = fcs*rectl_temp[2]*rectl_temp[3]/fcl/rects_temp[2]/rects_temp[3]
				t = (acs-acl)/255.0
				
				t = t*t
				#print(np.sum(t))
				cv2.putText(frameorg, "a"+str(((int(np.sum(t)*100)/100.0))), (cnt[0][0][0],cnt[0][0][1]), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

				if (d < 1 or fcl == 0) and np.sum(t) > 0.04:
					# print(vs)
					contourlist.append(cnt)
					cv2.drawContours(frameorg, [rectl, rects],-1, (0,255,0),1)

				#print(np.sum(acs-acl)/255)
				#print(d)
				#print(fcs*rectl_temp[2]*rectl_temp[3]/fcl/rects_temp[2]/rects_temp[3])
			#cv2.drawContours(frameorg, t[1],-1,(255,0,255),1)
			# print(len(list_currentkalman), len(list_matched), len(list_prevkalman))

			for cnt in contourlist:
				((cX, cY), radius) = cv2.minEnclosingCircle(cnt)
				cl = np.array([cX, cY])
				matched = False
				R = 0

				for (x, P), dc, mc, _ in list_prevkalman:
					br = False
					for (x2, P2) in list_matched:
						if (x==x2).all() and (P==P2).all():
							br = True
							break
					if br:
						continue
					lcv = P[:2, :2]
					t = np.array([cX-x[0][0], cY - x[1][0]]).reshape((1,2))
					lpp = np.matmul(np.matmul(t, lcv), t.T)
					if lpp < 5000:
						matched = True
						list_matched.append((x,P))
						list_currentkalman.append((kalman_xy(x, P, cl, R), dc+1, 0, cnt))
				
				if not matched:
					x = np.matrix('0. 0. 0. 0.').T 
					P = np.matrix(np.eye(4))*10 # initial uncertainty

					list_currentkalman.append((kalman_xy(x, P, cl, R), 1, 0, cnt))
			# print(len(list_currentkalman), len(list_matched), len(list_prevkalman))

			contourlist = []
			for (x, P), dc, mc, cnt in list_prevkalman:
				br=False
				for (x2, P2) in list_matched:
					if (x==x2).all() and (P==P2).all():
						br = True
						break
				if not br:
					if mc<10:
						list_currentkalman.append(((x, P), dc, mc+1, cnt))
				if dc>5:
					contourlist.append(cv2.boundingRect(cnt))
					cv2.circle(frameorg, (int(x[0][0]), int(x[1][0])), int(radius),
						(255, 0, 0), 3)
			list_prevkalman = list_currentkalman[:]
			cv2.imshow("f", frameorg)
			# image_pub.publish(bridge.cv2_to_imgmsg(frameorg, "bgr8"))

		if not contourlist:
			rate.sleep()
			continue
		p=0       # Keeping track of cnts published
		#[(x, y, 0), (x+w, y, 0), (x, y+h, 0), (x+h, y+h, 0)]
		for cnts in contourlist:
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

			# rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

point_publisher()
