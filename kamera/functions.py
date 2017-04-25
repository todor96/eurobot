import cv2
import numpy as np
import math
import os.path

CAM_DISTANCE = 20
OBJECT_RADIUS = 1
REAL_OBJECT_WIDTH = 1
REAL_OBJECT_HEIGHT = 1
REAL_OBJECT_DIAM = 1
RESOLUTION = (352,288)
lowSV = 100
highSV = 255

def rgb2hsvColor(rgb):
    bgr = np.uint8([[[rgb[2],rgb[1],rgb[0]]]])
    hsv = cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    return hsv[0][0]

def inputColorRGB():
    global lowSV, highSV

    print 'Input RBG value color'
    colorR = input('Color R value: ')
    colorG = input('Color G value: ')
    colorB = input('Color B value: ')
    print 'Input low and high value for SV'
    lowSV = input('LOW SV: ')
    highSV = input('HIGH SV: ')

    hsvColor = rgb2hsvColor([colorR,colorG,colorB])
    hsvLowerColor = hsvColor.copy()
    hsvUpperColor = hsvColor.copy()
    if(hsvLowerColor[0] >= 10):
        hsvLowerColor[0] -= 10
    else:
        hsvLowerColor[0] = 0

    if(hsvUpperColor[0] <= 245):
        hsvUpperColor[0] += 10
    else:
        hsvUpperColor[0] = 255

    hsvLowerColor[1] = hsvLowerColor[2] = lowSV
    hsvUpperColor[1] = hsvUpperColor[2] = highSV

    print 'LOW: ',hsvLowerColor,'HIGH: ',hsvUpperColor
    return {'upper-color':hsvUpperColor, 'lower-color':hsvLowerColor}

def calibrate(objRadius):
    global CAM_DISTANCE, OBJECT_RADIUS, REAL_OBJECT_WIDTH, REAL_OBJECT_HEIGHT, REAL_OBJECT_DIAM

    if(objRadius > 0):
        OBJECT_RADIUS = objRadius
        print '[+] Object height loaded succesfuly! Loaded height is', OBJECT_RADIUS
    else:
        print '[!] Invalid object height! Loaded height is', OBJECT_RADIUS
        return

    camDistance = input('Input distance form object: ')
    if(camDistance > 0):
        CAM_DISTANCE = camDistance
        print '[+] Distance from camera loaded succesfuly! Loaded distance is', CAM_DISTANCE
    else:
        print '[!] Invalid distance from camera! Loaded distance is', CAM_DISTANCE

    realObjWidth = input('Input real object width: ')
    if realObjWidth >= 0:
        REAL_OBJECT_WIDTH = realObjWidth
        print '[+] Real object width loaded succesfuly! Loaded real object width is', REAL_OBJECT_WIDTH
    else:
        print '[!] Invalid real object width! Loaded real object width is', REAL_OBJECT_WIDTH

    realObjHeight = input('Input real object height: ')
    if realObjHeight >= 0:
        REAL_OBJECT_HEIGHT = realObjHeight
        print '[+] Real object height loaded succesfuly! Loaded real object height is', REAL_OBJECT_HEIGHT
    else:
        print '[!] Invalid real object height! Loaded real object height is', REAL_OBJECT_HEIGHT

    REAL_OBJECT_DIAM = math.sqrt(REAL_OBJECT_WIDTH**2 + REAL_OBJECT_HEIGHT**2)

    calibrationFile = file('calibration.txt','w')
    calibrationFile.write(str(OBJECT_RADIUS) + ';' + str(CAM_DISTANCE) + ';' + str(REAL_OBJECT_WIDTH) + ';' + str(REAL_OBJECT_HEIGHT) + ';' + str(REAL_OBJECT_DIAM))
    calibrationFile.close()

def calibrateFromFile():
    global CAM_DISTANCE, OBJECT_RADIUS, REAL_OBJECT_WIDTH, REAL_OBJECT_HEIGHT, REAL_OBJECT_DIAM

    if(os.path.isfile('calibration.txt')):
        calibrationFile = file('calibration.txt','r')
        values = str(calibrationFile.readline()).split(';')
        calibrationFile.close()
        if len(values) != 5:
            print '[!] Invalid calibration file!'
            return

        OBJECT_RADIUS = float(values[0])
        CAM_DISTANCE = float(values[1])
        REAL_OBJECT_WIDTH = float(values[2])
        REAL_OBJECT_HEIGHT = float(values[3])
        REAL_OBJECT_DIAM = float(values[4])

        print '[+] Values from calibration file succesfuly loaded!'
    else:
        print '[!] Calibration file not found!'

def measureDistance(currentRadius):
    global CAM_DISTANCE, OBJECT_RADIUS

    if(currentRadius <= 0):
        print '[!] Invalid current height!'
        return

    proportion = currentRadius*1.0 / OBJECT_RADIUS
    currentDistance = CAM_DISTANCE / proportion

    return currentDistance

def measureAngle(center,currentRadius):
    global CAM_DISTANCE, REAL_OBJECT_DIAM

    screenCenter = (RESOLUTION[0]/2.0, RESOLUTION[1]/2.0)

    a = measureDistance(currentRadius)

    proportion = currentRadius*2/REAL_OBJECT_DIAM
    b = abs(center[0]-screenCenter[0])/proportion

    c = math.sqrt(a**2 + b**2)

    angle = 90 - math.degrees(math.acos(b/c*1.0))

    # direction = 1 -> RIGHT
    # direction = -1 -> LEFT
    direction = 1
    if(center[0] < screenCenter[0]):
        direction = -1

    return (angle,direction)
