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
from close_detect import *
# roi_detect OR close_detect OR close_detect_2

list_currentdetect = []
list_prevkalman = []
list_currentkalman = []
list_matched = []

list_currentdetect_2 = []
list_prevkalman_2 = []
list_currentkalman_2 = []
list_matched_2 = []

def IOU_area(cnt1, cnt2):
	[x1, y1, w1, h1] = cnt1
	[x2, y2, w2, h2] = cnt1
	total_area = (w1*h1)+(w2*h2)
	xA = max(x1,x2)
	yA = max(y1,y2)
	xB = max(x1+w1, x2+w2)
	yB = max(y1+h1, y2+h2)

	interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
	boxAArea = w1*h1
	boxBArea = w2*h2
	iou = interArea / float(boxAArea + boxBArea - interArea)

	# return the intersection over union value
	return iou

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
	contourlist_2, list_prevkalman_2 = cntsearch_2(frameorg, list_prevkalman_2)
	
	for cnts2 in contourlist_2:
		for cnts in contourlist:
			if iou(cnts, cnts2)>0.8:
				continue
			[x, y, w, h] = cnts2
			cv2.rectangle(frameorg, (x, y), (x + w, y + h), (0, 255, 0), 2)
			print("x is :" + str(x))
			print("y is :" + str(y))
			print("")
	cv2.imshow("contours", frameorg)
	cv2.waitKey(1)

