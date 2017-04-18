import cv2
import numpy as np
import sys
from functions import *
import functions
import time

PRINT_LOG = False
thresholdColor = {}
IMG_FRAME = None
enabledPick = False


def mouseCallback(event,x,y,flags,param):
    global thresholdColor,lowSV,highSV, enabledPick

    if not enabledPick:
        return

    if event == cv2.EVENT_LBUTTONDBLCLK:
        print '[+] Mouse position: x', x, 'y', y
        hsv = cv2.cvtColor(IMG_FRAME, cv2.COLOR_BGR2HSV)
        hsv_picked = hsv[x][y]
        print '[+] Picked color:', hsv_picked


        hsvLow_picked = hsv_picked.copy()
        hsvHigh_picked = hsv_picked.copy()

        if(hsvLow_picked[0] >= 10):
            hsvLow_picked[0] -= 10
        else:
            hsvLow_picked[0] = 0

        if(hsvHigh_picked[0] <= 245):
            hsvHigh_picked[0] += 10
        else:
            hsvHigh_picked[0] = 255

        hsvLow_picked[1] = hsvLow_picked[2] = lowSV
        hsvHigh_picked[1] = hsvHigh_picked[2] = highSV

        thresholdColor['lower-color'] = hsvLow_picked
        thresholdColor['upper-color'] = hsvHigh_picked

        print thresholdColor['lower-color'], thresholdColor['upper-color']

        enabledPick = False
    return



calibrateFromFile()
print functions.OBJECT_RADIUS,functions.CAM_DISTANCE,functions.REAL_OBJECT_WIDTH,functions.REAL_OBJECT_HEIGHT,functions.REAL_OBJECT_DIAM

cap = cv2.VideoCapture(1)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
# cap.set(cv2.cv.CV_CAP_PROP_FPS, 15)
cv2.namedWindow('frame')
cv2.setMouseCallback('frame',mouseCallback)




if len(sys.argv) > 1:

    for i in range(1,len(sys.argv)):
        if sys.argv[i] == '--rgb' and len(sys.argv)>=i+5:
            thresholdColor['lower-color'] = rgb2hsvColor([sys.argv[i+1], sys.argv[i+2], sys.argv[i+3]])
            thresholdColor['upper-color'] = rgb2hsvColor([sys.argv[i+1], sys.argv[i+2], sys.argv[i+3]])
            if(thresholdColor['lower-color'][0] >= 10):
                thresholdColor['lower-color'][0] -= 10
            else:
                thresholdColor['lower-color'][0] = 0

            if(thresholdColor['upper-color'][0] <= 245):
                thresholdColor['upper-color'][0] += 10
            else:
                thresholdColor['upper-color'][0] = 255

            thresholdColor['lower-color'][1] = thresholdColor['lower-color'][2] = int(sys.argv[i+4])
            thresholdColor['upper-color'][1] = thresholdColor['upper-color'][2] = int(sys.argv[i+5])

if not thresholdColor.has_key('lower-color') and not thresholdColor.has_key('upper-color'):
    thresholdColor = inputColorRGB()

while(True):

    if not enabledPick:
        ret, IMG_FRAME = cap.read()
        # IMG_FRAME = cv2.resize(IMG_FRAME,(640, 480), interpolation = cv2.INTER_CUBIC)
        hsv = cv2.cvtColor(IMG_FRAME, cv2.COLOR_BGR2HSV)


        lowerColor = np.array(thresholdColor['lower-color'])
        upperColor = np.array(thresholdColor['upper-color'])

        mask = cv2.inRange(hsv, lowerColor, upperColor)

        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        maxCntIndex = 0
        maxCntArea = 0

        for i in range(0,len(contours)):
            cntArea = cv2.contourArea(contours[i])
            if(cntArea > maxCntArea):
                maxCntIndex = i
                maxCntArea = cntArea
        circleRadius = 0

        if(maxCntArea > 0):
            (circleX, circleY), circleRadius = cv2.minEnclosingCircle(contours[maxCntIndex])
            circleRadius = int(circleRadius)
            circleCenter = (int(circleX),int(circleY))
            cv2.circle(IMG_FRAME,circleCenter,circleRadius, (0,0,255),3)
            # cv2.drawContours(frame, contours, maxCntIndex, (0,255,0), 3)

        cv2.imshow('frame',IMG_FRAME)

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
        cv2.imwrite(saveNamePic, IMG_FRAME)
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
    elif k == ord('p') or k == ord('P'):
        enabledPick = not enabledPick

cap.release()
cv2.destroyAllWindows()
