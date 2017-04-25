import cv2
import numpy as np
import sys
import functions
import time

PRINT_LOG = False
thresholdColor = {'lower-color':[0,0,0],'upper-color':[0,0,0]}
IMG_FRAME = None
enabledPick = False
pickedColorImg = np.zeros((200,200,3),np.uint8)
CAM_ID = 0


def mouseCallback(event,x,y,flags,param):
    global thresholdColor,lowSV,highSV, enabledPick, pickedColorImg, IMG_FRAME

    if not enabledPick:
        return

    if event == cv2.EVENT_LBUTTONDBLCLK:
        print '[+] Mouse position: x', x, 'y', y

        color = IMG_FRAME[y][x]
        colorT = (color[0],color[1],color[2])
        pickedColorImg[:] = colorT

        hsv = cv2.cvtColor(IMG_FRAME, cv2.COLOR_BGR2HSV)
        hsv_picked = hsv[y][x]
        print '[+] Picked color:', hsv_picked


        hsvLow_picked = hsv_picked.copy()
        hsvHigh_picked = hsv_picked.copy()

        hsvLow_picked[0] = hsvLow_picked[0]-5 if hsvLow_picked[0] >= 5 else 0
        hsvLow_picked[1] = hsvLow_picked[1]-50 if hsvLow_picked[1]>=50 else 0
        hsvLow_picked[2] = hsvLow_picked[2]-50 if hsvLow_picked[2]>=50 else 0

        hsvHigh_picked[1] = hsvHigh_picked[1]+5 if hsvHigh_picked[0] <= 250 else 255
        hsvHigh_picked[1] = hsvHigh_picked[1]+50 if hsvHigh_picked[1]<=205 else 255
        hsvHigh_picked[2] = hsvHigh_picked[2]+50 if hsvHigh_picked[2]<=205 else 255

        thresholdColor['lower-color'] = hsvLow_picked
        thresholdColor['upper-color'] = hsvHigh_picked

        print thresholdColor['lower-color'], thresholdColor['upper-color']

        enabledPick = False
    return



if len(sys.argv) > 1:
    for i in range(1,len(sys.argv)):
        if sys.argv[i] == '--rgb':
            thresholdColor = inputColorRGB()
        elif sys.argv[i] == '--cam' and len(sys.argv)>=i+1:
            CAM_ID = int(sys.argv[i+1])
            print '[+] Camera set to', CAM_ID



functions.calibrateFromFile()
print functions.OBJECT_RADIUS,functions.CAM_DISTANCE,functions.REAL_OBJECT_WIDTH,functions.REAL_OBJECT_HEIGHT,functions.REAL_OBJECT_DIAM

cap = cv2.VideoCapture(CAM_ID)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, functions.RESOLUTION[0])
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, functions.RESOLUTION[1])
# cap.set(cv2.cv.CV_CAP_PROP_FPS, 15)
cv2.namedWindow('frame')
cv2.setMouseCallback('frame',mouseCallback)


# frames_num = 120
# start_time = time.time()


while(True):
    # frames_num -= 1

    if not enabledPick:
        ret, IMG_FRAME = cap.read()
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
        cv2.imshow('mask',mask)
        cv2.imshow('picked',pickedColorImg)

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
        currDistance = functions.measureDistance(circleRadius)
        print '[+] Distance:', currDistance, 'cm'
    elif k == ord('c') or k == ord('C'):
        functions.calibrate(circleRadius)
    elif k == ord('l') or k == ord('L'):
        PRINT_LOG = not PRINT_LOG
    elif k == ord('a') or k == ord('A'):
        currAngle, currDirection = functions.measureAngle(circleCenter,circleRadius)
        print '[+] Angle:', currAngle, ' Direction:', currDirection
    elif k == ord('p') or k == ord('P'):
        enabledPick = not enabledPick



# end_time = time.time()
# seconds = end_time - start_time
# print 'SECS',seconds
# fps = 120/seconds
# print 'FPS',fps

cap.release()
cv2.destroyAllWindows()
