import numpy as np
import cv2

img = cv2.imread('test.jpg',-1)
imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

ret, thresh = cv2.threshold(imgray,127,255,0)
con, hier = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# cv2.drawContours(img, con, 1, (0,255,0), 3)

# x,y,w,h = cv2.boundingRect(con[4])
# cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)

while True:
    cv2.imshow('img',img)



    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
