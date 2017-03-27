import sys
import numpy as np
import cv2

if len(sys.argv) < 4:
    print "Nedovoljno argumenata"
    exit(0)

blue = sys.argv[1]
green = sys.argv[2]
red = sys.argv[3]

bgr = np.uint8([[[blue,green,red]]])
hvs = cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)

print hvs
