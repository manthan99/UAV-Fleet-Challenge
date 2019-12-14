import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2
import numpy as np 

from imutils import contours
from skimage import measure
import numpy as np
import argparse
import imutils

# import rospy
# from std_msgs.msg import String
# from geometry_msgs.msg import Point
# from detection.msg import point_list
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

from close_detect_2 import *
# roi_detect OR close_detect OR close_detect_2

list_currentdetect = []
list_prevkalman = []
list_currentkalman = []
list_matched = []

cap = cv2.VideoCapture("/home/debjoy/A_projects/interiit/architwalacode/2018_0106_202440_002.MOV")

if (cap.isOpened() == False):
	print("Unable to read feed")

while True:
	_, frameorg = cap.read()

	image = frameorg.copy()
	frameorg = cv2.resize(frameorg, (image.shape[1]//2, image.shape[0]//2))
	contourlist = []
	cv2.imshow("init", frameorg)
	contourlist, list_prevkalman = cntsearch(frameorg, list_prevkalman)
	for cnts in contourlist:
		[x, y, w, h] = cnts
		cv2.rectangle(frameorg, (x, y), (x + w, y + h), (0, 255, 0), 2)
		print("x is :" + str(x))
		print("y is :" + str(y))
		print("")
	cv2.imshow("contours", frameorg)
	cv2.waitKey(1)

