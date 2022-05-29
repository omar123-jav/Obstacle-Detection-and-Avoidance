import numpy as np

from vis_image import vis_image
from vis_points import vis_points
from vis_points import vis_points2
from views import *
import cv2
import time
import sys

start = time.time()



frame = '000032'
img_path = 'data/' + frame+ '.png'
pts_path = 'data/' + frame+'.bin'
calib_path = 'data/' +'c.txt'
label_path = 'data/' + '000032' + '_label.txt'

# define output path for image visualisation and create image

arr=np.array([[0,1,2],[3,4,5],[6,7,8],[9,10,11]])
print(arr.shape)
# define output path for points visualisation and create image
output_path = "output/vis_points.png"
img = vis_points2(
    output_path,
    pts_path,
    label_path,
    calib_path,
    boxes_3d=False,
    view_dict=BEV
)




end = time.time()

print("The time of execution of above program is :", end-start)