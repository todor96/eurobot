import cv2
import numpy as np
import sys
from functions import *
import time

PRINT_LOG = False

thresholdColor = {}
cap = cv2.VideoCapture(0)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
cap.set(cv2.cv.CV_CAP_PROP_FPS, 15)

if len(sys.argv) > 1:
    for i in range(1,len(sys.argv)):
        if sys.argv[i] == '--low-rgb' and len(sys.argv)>=i+3:
            thresholdColor['lower-color'] = rgb2hsvColor([sys.argv[i+1], sys.argv[i+2], sys.argv[i+3]])
        elif sys.argv[i] == '--up-rgb' and len(sys.argv)>=i+3:
            thresholdColor['upper-color'] = rgb2hsvColor([sys.argv[i+1], sys.argv[i+2], sys.argv[i+3]])
        elif sys.argv[i] == '--low-hsv' and len(sys.argv)>=i+3:
            thresholdColor['lower-color'] = [int(sys.argv[i+1]),int(sys.argv[i+2]),int(sys.argv[i+3])]
        elif sys.argv[i] == '--up-hsv' and len(sys.argv)>=i+3:
            thresholdColor['upper-color'] = [int(sys.argv[i+1]),int(sys.argv[i+2]),int(sys.argv[i+3])]

if not thresholdColor.has_key('lower-color') and not thresholdColor.has_key('upper-color'):
    thresholdColor = inputColorRGB()

while(True):
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lowerColor = np.array(thresholdColor['lower-color'])
    upperColor = np.array(thresholdColor['upper-color'])

    mask = cv2.inRange(hsv, lowerColor, upperColor)


    # ret, thresh = cv2.threshold(mask,127,255,0)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # result = cv2.bitwise_and(frame, frame, mask=mask)

    # cv2.imshow('frame',frame)
    # cv2.imshow('result',result)

    maxCntIndex = 0
    maxCntArea = 0

    for i in range(0,len(contours)):
        cntArea = cv2.contourArea(contours[i])
        if(cntArea > maxCntArea):
            maxCntIndex = i
            maxCntArea = cntArea
    # x = y = w = h = 0
    circleRadius = 0

    if(maxCntArea > 0):
        # x,y,w,h = cv2.boundingRect(contours[maxCntIndex])
        # cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 3)

        (circleX, circleY), circleRadius = cv2.minEnclosingCircle(contours[maxCntIndex])
        circleRadius = int(circleRadius)
        circleCenter = (int(circleX),int(circleY))
        cv2.circle(frame,circleCenter,circleRadius, (0,0,255),3)
        # cv2.drawContours(frame, contours, maxCntIndex, (0,255,0), 3)

    cv2.imshow('mask',frame)

    #log
    # print '[+] LC:', thresholdColor['lower-color'], 'UC:', thresholdColor['upper-color'], 'LEN:', len(contours), 'MAX:', maxCntIndex
    if(PRINT_LOG):
        print '[+] CONTOURS:', len(contours), 'MAX:', maxCntIndex, 'RADIUS:',circleRadius

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    elif k == ord('s') or k == ord('S'):
        saveNamePic = 'save' + time.strftime('%y-%m-%d') + '-' + time.strftime('%H-%M-%S') + '.jpg'
        saveNameLog = 'save' + time.strftime('%y-%m-%d') + '-' + time.strftime('%H-%M-%S') + '.log'
        print '[+] Saving... ', saveNamePic, saveNameLog
        saveLog = file(saveNameLog, 'w')
        saveLog.write('[+] CONTOURS: ' + str(len(contours)) + ' MAX: ' + str(maxCntIndex) + ' RADIUS: ' + str(circleRadius))
        saveLog.close()
        cv2.imwrite(saveNamePic, frame)
    elif k == ord('m') or k == ord('M'):
        currDistance = measureDistance(circleRadius)
        print '[+] Distance:', currDistance, 'cm'
    elif k == ord('c') or k == ord('C'):
        calibrate(circleRadius)
    elif k == ord('l') or k == ord('L'):
        PRINT_LOG = not PRINT_LOG
    elif k == ord('a') or k == ord('A'):
        currAngle, currDirection = measureAngle(circleCenter,circleRadius)
        print '[+] Angle:', currAngle, ' Direction:', currDirection

cap.release()
cv2.destroyAllWindows()
