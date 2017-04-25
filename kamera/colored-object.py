import cv2
import numpy as np
import sys
import functions
import time
import os

#globalne promenljive
PRINT_LOG = False
thresholdColor = {'lower-color':[0,0,0],'upper-color':[0,0,0]}      #donja i gornja granica boja za izdvajanje
IMG_FRAME = None                                                    #frejm koji trenutno ucitan sa kamere
enabledPick = False                                                 #ukljucivanje rezima za biranje boje sa frejma
pickedColorImg = np.zeros((200,200,3),np.uint8)                     #slika ispunjena bojom koja je izabrana
CAM_ID = 0                                                          #id kamere koja se koristi
SHOW_WINDOWS = False                                                #prikazivanje prozora za pracenje rada programa
TEST_FPS = False                                                    #testiranje fpsa
COLOR_FROM_FILE = False                                             #automatsko biranje boja za izdvajanje iz fajla


#funkcija uzima gornju i donju granicu boje za izdvajanje
#smesta vrednosti u globalnu 'thresholdColor'
def getColorFromFile():
    global thresholdColor
    if(os.path.isfile('colors.txt')):
        colorsFile = file('colors.txt','r')
        values = str(colorsFile.readline()).split(';')
        colorsFile.close()
        if len(values) != 6:
            print '[!] Invalid colors file!'
            return

        thresholdColor['lower-color'][0] = int(values[0])
        thresholdColor['lower-color'][1] = int(values[1])
        thresholdColor['lower-color'][2] = int(values[2])
        thresholdColor['upper-color'][0] = int(values[3])
        thresholdColor['upper-color'][1] = int(values[4])
        thresholdColor['upper-color'][2] = int(values[5])

        print '[+] Values from colors file succesfuly loaded!'
    else:
        print '[!] Colors file not found!'

#callback funkcija za frame prozor
#omogucava biranje boje
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

        #write in file
        colorsFile = file('colors.txt','w')
        str1 = ';'.join(str(v) for v in hsvLow_picked)
        str2 = ';'.join(str(v) for v in hsvHigh_picked)
        colorsFile.write(str1 + ';' + str2)
        colorsFile.close
        print '[+] Color saved in colors file!'
        #end write in file

        thresholdColor['lower-color'] = hsvLow_picked
        thresholdColor['upper-color'] = hsvHigh_picked

        print thresholdColor['lower-color'], thresholdColor['upper-color']

        enabledPick = False
    return


#provera argumenata komandne linije
#   --cam 'id'-> postavlja 'CAM_ID' na 'id'
#   --show-windows -> postavlja 'SHOW_WINDOWS' na True
#   --log -> postavlja 'PRINT_LOG' na True
#   --test-fps -> postavlja 'TEST_FPS' na True
#   --color-file -> postavlja 'COLOR_FROM_FILE' na True
#   --rgb -> omogucava unos rgb boje i donje i gornje granice za hsv
if len(sys.argv) > 1:
    for i in range(1,len(sys.argv)):
        if sys.argv[i] == '--rgb':
            thresholdColor = inputColorRGB()
        elif sys.argv[i] == '--cam' and len(sys.argv)>=i+1:
            CAM_ID = int(sys.argv[i+1])
            print '[+] Camera set to', CAM_ID
        elif sys.argv[i] == '--show-windows':
            SHOW_WINDOWS = True
        elif sys.argv[i] == '--log':
            PRINT_LOG = True
        elif sys.argv[i] == '--test-fps':
            TEST_FPS = True
        elif sys.argv[i] == '--color-file':
            COLOR_FROM_FILE = True

if COLOR_FROM_FILE:
    getColorFromFile()

#kalibracija iz fajla i ispis preuzetih vrednosti
functions.calibrateFromFile()
print functions.OBJECT_RADIUS,functions.CAM_DISTANCE,functions.REAL_OBJECT_WIDTH,functions.REAL_OBJECT_HEIGHT,functions.REAL_OBJECT_DIAM

#pravljenje i konfigurisanje VideoCapture objekta
cap = cv2.VideoCapture(CAM_ID)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, functions.RESOLUTION[0])
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, functions.RESOLUTION[1])
# cap.set(cv2.cv.CV_CAP_PROP_FPS, 15)

#uslovno kreiranje prozora
if SHOW_WINDOWS:
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame',mouseCallback)

#postavljanje vrednosti za test-fps
frames_num = 120
start_time = None

#uslovno startovanje test-fpsa
if TEST_FPS:
    start_time = time.time()

#beskonacna petlja u slucaju da nije ukljucen 'TEST_FPS'
#ako je 'TEST_FPS' ukljucen petlja ima 'frames_num' iteracija
while(frames_num > 0):
    if TEST_FPS:
        frames_num -= 1

    #ako se trenutno ne bira boja
    if not enabledPick:
        ret, IMG_FRAME = cap.read()                             #ucitavanje frejma iz kamere
        hsv = cv2.cvtColor(IMG_FRAME, cv2.COLOR_BGR2HSV)        #prebacivanje frejma iz BGR u HSV


        lowerColor = np.array(thresholdColor['lower-color'])    #donja granica boja
        upperColor = np.array(thresholdColor['upper-color'])    #gornja granica boja

        mask = cv2.inRange(hsv, lowerColor, upperColor)         #maska od piskela koji spadaju u opseg boja

        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)      #pronalazenje kontura na masci

        maxCntIndex = 0     #indeks najvece konture
        maxCntArea = 0      #povrsina najvece konture

        #pronalazenje najvece konture u listi kontura
        for i in range(0,len(contours)):
            cntArea = cv2.contourArea(contours[i])
            if(cntArea > maxCntArea):
                maxCntIndex = i
                maxCntArea = cntArea

        circleRadius = 0        #precnik opisanog kruga oko najvece konture
        circleCenter = None     #centar opisanog kruga oko najvece konture

        #pornalazenje centra i precnika opisanog kruga oko najvece konture
        if(maxCntArea > 0):
            (circleX, circleY), circleRadius = cv2.minEnclosingCircle(contours[maxCntIndex])
            circleRadius = int(circleRadius)
            circleCenter = (int(circleX),int(circleY))
            cv2.circle(IMG_FRAME,circleCenter,circleRadius, (0,0,255),3)        #isrtavnaje kruga na frejmu sa kamere
            # cv2.drawContours(frame, contours, maxCntIndex, (0,255,0), 3)

        #uslovno prikazivanje frejmova na prozore
        if SHOW_WINDOWS:
            cv2.imshow('frame',IMG_FRAME)
            cv2.imshow('mask',mask)
            cv2.imshow('picked',pickedColorImg)

    #uslovno ispisivanje log-a
    if(PRINT_LOG):
        print '[+] CONTOURS:', len(contours), 'MAX:', maxCntIndex, 'RADIUS:',circleRadius, 'CENTER:', circleCenter

    #uslovno osluskivanje na pritisak sa tastature za prozor 'frame'
    if SHOW_WINDOWS:
        k = cv2.waitKey(1) & 0xFF
        if k == 27:                                 #prekidanje rada programa
            break
        elif k == ord('s') or k == ord('S'):        #cuvanje loga u vidu trentnog frejma i log fajla
            saveNamePic = 'save' + time.strftime('%y-%m-%d') + '-' + time.strftime('%H-%M-%S') + '.jpg'
            saveNameLog = 'save' + time.strftime('%y-%m-%d') + '-' + time.strftime('%H-%M-%S') + '.log'
            print '[+] Saving... ', saveNamePic, saveNameLog
            saveLog = file(saveNameLog, 'w')
            saveLog.write('[+] CONTOURS: ' + str(len(contours)) + ' MAX: ' + str(maxCntIndex) + ' RADIUS: ' + str(circleRadius))
            saveLog.close()
            cv2.imwrite(saveNamePic, IMG_FRAME)
        elif k == ord('m') or k == ord('M'):        #merenje distance do objekta u cm
            currDistance = functions.measureDistance(circleRadius)
            print '[+] Distance:', currDistance, 'cm'
        elif k == ord('c') or k == ord('C'):        #vrsenje kalibracije
            functions.calibrate(circleRadius)
        elif k == ord('l') or k == ord('L'):        #ukljucivanje 'PRINT_LOG'
            PRINT_LOG = not PRINT_LOG
        elif k == ord('a') or k == ord('A'):        #merenje ugla objekta u odnosu na centar kamere
            currAngle, currDirection = functions.measureAngle(circleCenter,circleRadius)
            print '[+] Angle:', currAngle, ' Direction:', currDirection
        elif k == ord('p') or k == ord('P'):        #biranje boje za izdvajanje
            enabledPick = not enabledPick


#uslovno testiranje fps-a
if TEST_FPS:
    end_time = time.time()
    seconds = end_time - start_time
    print 'SECS',seconds
    fps = 120/seconds
    print 'FPS',fps

#gasenje VideoCapture objekta i gasenje svih prozora
cap.release()
cv2.destroyAllWindows()
