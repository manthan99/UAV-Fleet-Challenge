#!/usr/bin/env python


# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")

import cv2 
import numpy as np 

import argparse
import close_detect_help
from imutils import contours
from skimage import measure
import imutils
font = cv2.FONT_HERSHEY_COMPLEX


# from cv_bridge import CvBridge, CvBridgeError

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

def avg_col(img):
    rows = img.shape[0]
    cols = img.shape[1]
    colour = np.int32([np.sum(img[:, :, 0]), np.sum(img[:, :, 1]), np.sum(img[:, :, 2])])/(rows*cols)
    return colour 

def avg_col_bw(img):
    rows = img.shape[0]
    cols = img.shape[1]
    colour = np.int32(np.sum(img[:, :]))/(rows*cols)
    return colour 

def avg_col_removed(img, img0):
    rows = img.shape[0]
    cols = img.shape[1]
    rows0 = img0.shape[0]
    cols0 = img0.shape[1]
    colour = (np.int32([np.sum(img[:, :, 0]), np.sum(img[:, :, 1]), np.sum(img[:, :, 2])])-np.int32([np.sum(img0[:, :, 0]), np.sum(img0[:, :, 1]), np.sum(img0[:, :, 2])]))/((rows*cols)-(rows0*cols0))
    return colour 

def adjust_gamma(image, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
 
    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)

def dist(pt1, pt2):
    return np.sqrt((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)

def cntsearch(frameorg, state):
    list_prevkalman = state
    list_currentkalman = []
    list_matched = []
    frame = frameorg
    image = frame[:1000, :, :]
    image = adjust_gamma(image, 1)
    # # DID resize
    # image = cv2.resize(image, (image.shape[1]//2, image.shape[0]//2))
    
    threshold= cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (25, 0, 0), (100, 255, 255));  # broad threshold on hue
    cv2.imshow("inRange", threshold)

    cv2.imshow("frame",image)
    blur = (cv2.GaussianBlur(image, (5,5), 0))
    
    #grayg = image[:, :, 1]
    #blurs = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    # blurs = blur[:, :, 1]
    # cv2.imshow("blurgreen", blurs)
    # cv2.imshow
    # blur = grayg
    
    # blurred = blur[:, :, 1]
    #blurred = blur
    # blurred = cv2.equalizeHist(blurred)
    # cv2.imshow("blurgray", blurs)
    # ret,thresh = cv2.threshold(blurs,150,255,0)
    # thresh = cv2.erode(thresh, None, iterations=6)
    # thresh = cv2.dilate(thresh, None, iterations=6)
    # cv2.imshow("thresherodedilate", thresh)
    # cv2.imshow("thresh", thresh)

    imgx = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    imgy = cv2.cvtColor(imgx, cv2.COLOR_BGR2GRAY)

    ret, threshhsv = cv2.threshold(imgy,100,255,0)            #######param for threshold, hsv ko rgb treat krke gray mein convert kra
    cv2.imshow("invthresh", threshhsv)
    
    thresh = np.uint8(threshold*(threshold/255.0))
    threshhsv = np.uint8(threshold*(threshhsv/255.0))

    _, invcontours, invhierarchy = cv2.findContours((threshhsv),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # cnts = cv2.findContours(gray.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # contours = imutils.grab_contours(contours)
    cnts = []
    # for cnt in contours:
    #   k = cv2.isContourConvex(cnt)
    #   # if not k:
    #   #   continue
    #   area = cv2.contourArea(cnt)
    #   if area<300 or area>2000:
    #       continue

    #   epsilon = 0.05*cv2.arcLength(cnt,True)
    #   approx = cv2.approxPolyDP(cnt,epsilon,True)
    #   if len(approx) != 4:
    #       continue
        
    #   hull = cv2.convexHull(cnt)
    #   hullarea = cv2.contourArea(hull)
    #   if hullarea != 0:
    #       # print(area/hullarea)
    #       if area/hullarea<0.7:
    #           continue

    #   rect = cv2.minAreaRect(cnt)
    #   rectarea = cv2.contourArea(hull)
    #   # print(rectarea/area)
    #   # if rectarea/area>1.10:
    #   #   continue

    #   cnts.append(cnt)

    for cnt in invcontours:
        # k = cv2.isContourConvex(cnt)
        # if not k:
        #   continue
        area = cv2.contourArea(cnt)
        if area<200 or area>50000:
            print("less area")
            continue
        print("area: "+str(area))


        epsilon = 0.05*cv2.arcLength(cnt,True)     # param might not be changed
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        if len(approx) != 4:
            print("not box")
            continue
        
        hull = cv2.convexHull(cnt)
        hullarea = cv2.contourArea(hull)
        # print(area/hullarea)
        # if hullarea!=0:
        #   if area/hullarea<0.7:
        #       continue
        rect = cv2.minAreaRect(cnt)
        # rectarea = cv2.contourArea(rect)
        box = cv2.boxPoints(rect)

        print(box)
        ar = dist(box[0], box[1])/dist(box[2], box[1])
        # if ar>1.3 or ar<0.7:
        #   print("ar: "+str(ar))
        #   continue
        # print(rectarea/area)

        cnts.append(cnt)

    cnts1 = []
    for cnt in cnts:
        (x, y, w, h) = cv2.boundingRect(cnt)

        cntimg = blur[y:y+h, x:x+w, :]
        cnt_extnd_wnd = blur[max(0, y-10):min(y+h+10, image.shape[0]), max(0, x-10):min(x+w+10, image.shape[1]), :]
        c1 = avg_col(cntimg)
        c0 = avg_col_removed(cnt_extnd_wnd, cntimg)
        diffs = (np.mean(np.abs(c1 - c0)))
        if diffs<20:
            # print("GRASS")
            print("GRASS")
            continue
        threshimg = threshold[y:y+h, x:x+w]
        threscol = avg_col_bw(threshimg)
        print(threscol)
        if threscol<150:
            print("not green")
            continue
        cnts1.append(cnt)
        
    for cnt in cnts1:
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
    # print(cnts1)
    fcontours = []
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
            fcontours.append(cv2.boundingRect(cnt))
            cv2.circle(image, (int(x[0][0]), int(x[1][0])), int(10),
                (0, 255, 0), 3)
    cv2.imshow("image", image)
    list_prevkalman = list_currentkalman[:]
    return fcontours, list_prevkalman
