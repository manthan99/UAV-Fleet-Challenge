#!/usr/bin/env python
import sys
print(sys.path)

#from __future__ import (absolute_import, division, print_function, unicode_literals)
import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Point
from detection.msg import point_list, local_flags

# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")

import cv2 
import numpy as np 

import numpy as np
import argparse

import cv2
font = cv2.FONT_HERSHEY_COMPLEX
import roi_detect
import close_detect

current_flag = 1
state = []
def callback(data):
	global state, current_flag
	if data.scan_flag.data == True:
		new_flag = 1
	elif data.search_flag.data == True:
		new_flag = 2
	else:
		new_flag = 0

	if new_flag!=current_flag:
		state = []
	current_flag=new_flag

def point_publisher():
	global state, current_flag

	################## CHANGE VIDEO FOR CAMERA Stream
	cap = cv2.VideoCapture("/home/archit/Documents/INter IIT/b.MOV")

	if (cap.isOpened() == False): 
		print("Unable to read feed")
	rospy.init_node('detect_ros', anonymous=True)

	pub = rospy.Publisher('roi_coordinates', point_list)
	pubc = rospy.Publisher('close_coordinates', point_list)
	rospy.Subscriber("/drone0/flag", local_flags, callback)

	rate = rospy.Rate(60)
	msg = point_list()

	while not rospy.is_shutdown():
		_,frameorg = cap.read()

		if current_flag == 0:
			rate.sleep()
			continue
		contourlist = []
		cv2.imshow("init", frameorg)
		if current_flag == 1:
			contourlist, state = roi_detect.cntsearch(frameorg, state)
		else:
			contourlist, state = close_detect.cntsearch(frameorg, state)
		for cnts in contourlist:
			[x, y, w, h] = cnts
			cv2.rectangle(frameorg, (x,y), (x+w,y+h), (0,255,0), 2)

			point1 = Point()
			point1.x = x+w/2.0
			point1.y = y+h/2.0
			point1.z = 0
			msg.points.append(point1)
		cv2.imshow("f", frameorg)
		cv2.waitKey(1)
		if current_flag==1:
			pub.publish(msg)
		else:
			pubc.publish(msg)
		msg =point_list()
		rate.sleep()

point_publisher()
