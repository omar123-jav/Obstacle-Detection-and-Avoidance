import numpy as np
import cv2
from reporjections import *
from load_data import *
from views import *
import copy


# reproject and draw point cloud and 3d bounding boxes onto a black background
def vis_points(
    output_path,
    pts_path,
    label_path,
    calib_path,
    boxes_3d=True,
    view_dict=TPV
):

    # read in points and bounding boxes
    points = read_pts(pts_path)

    # create black background
    IMG_H = 1500
    IMG_W = 3000
    img = np.zeros((IMG_H, IMG_W, 3), np.uint8)

    # create transformation matrix

    
    # reproject point cloud onto image
    img=birds_eye_point_cloud(points,
                          side_range=(-20, 20),
                          fwd_range=(0, 20),
                          res=0.1,
                          min_height=-1.25,
                          max_height=1.5,
                          saveto=None)

    _, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)
     #reporject 3d bounding boxes onto image
    kernel = np.ones((5, 5), np.uint8)
    dilate = cv2.dilate(thresh, kernel, iterations=1)


    image = np.uint8(img)
    speed=2.5

    contours, hierarchy = cv2.findContours(image=dilate, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    map = np.zeros(img.shape)
    nearestobstacles = []
    for i in contours:
        area = cv2.contourArea(i)
        if area > 500:
            nearestobstacles.append(i)
    mindistance = 100000000000000000000000
    mintheta=100000000
    obstaclearea=1000


    avoid=False
    drift=90
    distances=[]
    thetas=[]
    idist=-1
    itheta=-1
    count=0
    imin = -1
    for i in nearestobstacles:
        moment = cv2.moments(i)  # get the centroid of the obstacle using its moment
        cx = int(moment['m10'] / moment['m00'])
        cy = int(moment['m01'] / moment['m00'])
        cx = cx - 200
        cy = -1 * (cy - 200)
        cx = cx * 0.1
        cy = cy * 0.1
        if cx == 0:
            cx = cx + 0.001
        x2 = (cx) ** 2
        y2 = (cy) ** 2
        add = x2 + y2
        div = cy / cx
        theta = np.arctan(div)
        dist = np.sqrt(add)
        if(0<theta<5):
            if(avoid):
                drift=90
                avoid=False
        theta=(theta*180)/(np.pi)
        if (theta < 0):
            theta = theta + 180
        print(theta)
        if 75<theta<105:
            distances.append(dist)
            thetas.append(theta)
            idist+=1
            count += 1
            if dist<mindistance:
                mindistance = dist
                imin=idist
                mintheta=theta
                obstaclearea=cv2.contourArea(i)


        r2 = obstaclearea / np.pi
        width = np.sqrt(r2)
    if 4<= mindistance <= 10:
            avoid = True
            i=0
            right=False
            left=False
            for d in distances,thetas:
                if right & left:
                    break
                if i!=imin:
                    if(np.abs(d[0]-mindistance)<=2):
                        if(d[1]>mintheta):
                            left=True
                        else:
                            right=True
                i+=1
            if(left&right):
                speed=0
            elif(left):

                drift=mintheta-np.arctan(mindistance/width)+3
            elif(right):
                drift=mintheta+np.arctan(mindistance/width)+3
            else:
                drift = mintheta - np.arctan(mindistance / width)+3
    elif mindistance<4:
          speed=0
    elif count < 2:
        if avoid == False:
            speed = 5
    print("speed:",speed)
    print(count)
    print("drift:",drift)
    print("avoidance:",avoid)
    print("mindistance",mindistance)
    print(count)



def vis_points2(
        output_path,
        pts_path,
        label_path,
        calib_path,
        boxes_3d=True,
        view_dict=TPV
):
    # read in points and bounding boxes
    points = read_pts(pts_path)

    # create black background
    IMG_H = 1500
    IMG_W = 3000
    img = np.zeros((IMG_H, IMG_W, 3), np.uint8)

    # create transformation matrix

    # reproject point cloud onto image
    img = birds_eye_point_cloud(points,
                                side_range=(-20, 20),
                                fwd_range=(0, 20),
                                res=0.1,
                                min_height=-1.25,
                                max_height=1.5,
                                saveto=output_path)














