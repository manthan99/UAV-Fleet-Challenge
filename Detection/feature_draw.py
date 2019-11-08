import numpy as np
import cv2

cap = cv2.VideoCapture('2019_1102_113410_003.MOV')
sift = cv2.xfeatures2d.SIFT_create()

cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
print("a")
while(cap.isOpened()):
    _, img = cap.read()
    if cv2.waitKey(1) & 0xFF == ord('a'):
        gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        kp = sift.detect(gray,None)

        cv2.drawKeypoints(gray,kp,img)

    cv2.imshow('frame',img)



cap.release()
cv2.destroyAllWindows()
