import cv2
import numpy as np



img=cv2.imread('output/vis_points.png')
_, thresh = cv2.threshold(img,0,255,cv2.THRESH_BINARY)
cv2.imwrite('output/thr.png',thresh)









