import cv2
import numpy as np


from depth_image_processing import *
thresh=cv2.imread('output/thr.png')
thresh=cv2.cvtColor(thresh,cv2.COLOR_BGR2GRAY)
kernel = np.ones((5, 5), np.uint8)
dilate=cv2.dilate(thresh,kernel,iterations=1)

image=cv2.imread('output/vis_points.png')
img= np.uint8(image)
contours, hierarchy = cv2.findContours(image=dilate, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
image_copy = image.copy()
color=cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2,
                 lineType=cv2.LINE_AA)

map=np.zeros(image.shape)
nearestobstacles=[]

for i in contours:
    area=cv2.contourArea(i)
    if area>500:
        nearestobstacles.append(i)

map=np.zeros(image.shape)
obstacles=cv2.drawContours(map,nearestobstacles,-1,(0,255,0),thickness=2,lineType=cv2.LINE_AA)
cv2.imwrite('output/obstacles.png',obstacles)
ob=cv2.circle(obstacles,(200,130),20,(255,0,0),2)
ob=cv2.circle(obstacles,(250,140),20,(255,0,0),2)
ob_c=np.uint8(ob)
obthr=cv2.cvtColor(ob_c,cv2.COLOR_BGR2GRAY)
contours, hierarchy = cv2.findContours(image=obthr, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
color=cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2,
                 lineType=cv2.LINE_AA)
cv2.imwrite('output/obstacles.png',color)

mindistance=100000000000000000000000
mintheta=100000000000000000000
obstaclearea = 1000

avoid = False
drift = 90
speed=2.5
distances = []
thetas = []
idist = -1
itheta = -1
count = 0
imin = -1
for i in contours:
    moment = cv2.moments(i)  # get the centroid of the obstacle using its moment
    cx = int(moment['m10'] / moment['m00'])
    cy = int(moment['m01'] / moment['m00'])
    cx=cx-200
    cy=-1*(cy-200)
    cx=cx*0.1
    cy=cy*0.1
    if cx==0:
        cx=cx+0.001
    print(cx,cy)
    x2=(cx)**2
    y2=(cy)**2
    add=x2+y2
    div=cy/cx
    theta=np.arctan(div)

    dist=np.sqrt(add)
    theta=(theta*180)/np.pi
    if (theta < 0):
        theta = theta + 180
    print(theta)
    if 75<theta<105:
        distances.append(dist)
        thetas.append(theta)
        idist+=1
        count =count+ 1

        if dist<mindistance:
                mindistance = dist
                imin=idist
                mintheta=theta
                obstaclearea=cv2.contourArea(i)
    r2 = obstaclearea / np.pi
    width = np.sqrt(r2)
    print(width)
if 4 <= mindistance <= 10:
    avoid = True
    i = 0
    right = False
    left = False
    for d in distances, thetas:

        if i != imin:
            if (np.abs(d[0] - mindistance) <= 2):
                if (d[1] > mintheta):
                    left = True
                else:
                    right = True

        i += 1
    if (left & right):
        speed = 0
    elif (left):

        drift = np.arctan(mindistance / width)
        drift=drift*180
        drift=drift/np.pi
        drift=mintheta-drift-7
    elif (right):
        drift = np.arctan(mindistance / width)
        drift = drift * 180
        drift = drift / np.pi
        drift=drift+mintheta+7
    else:
        drift = np.arctan(mindistance / width)
        drift = drift * 180
        drift = drift / np.pi
        drift=mintheta - drift - 7
elif mindistance < 4:
    speed = 0
elif  count < 2:
    if avoid==False:
          speed=5
print("speed:", speed)
print(count)
print("drift:", drift)
print("avoidance:", avoid)
print("mindistance", mindistance)