import math
import numpy as np
import cv2
from PIL import Image

def scale_to_255(a, min, max, dtype=np.uint8):
    return (((a - min) / float(max - min)) * 255).astype(dtype)
def scale(a, min, max, dtype=np.uint8):
    return (255)
def birds_eye_point_cloud2(points,
                          side_range=(-10, 10),
                          fwd_range=(-10,10),
                          res=0.1,
                          min_height = -0.5,
                          max_height = 2,
                          saveto=None):


    x_lidar = points[:, 0]
    y_lidar = points[:, 1]
    z_lidar = points[:, 2]
    print(x_lidar)
    # r_lidar = points[:, 3]  # Reflectance

    # INDICES FILTER - of values within the desired rectangle
    # Note left side is positive y axis in LIDAR coordinates
    ff = np.logical_and((x_lidar > fwd_range[0]), (x_lidar < fwd_range[1]))
    ss = np.logical_and((y_lidar > -side_range[1]), (y_lidar < -side_range[0]))
    indices = np.argwhere(np.logical_and(ff,ss)).flatten()

    # CONVERT TO PIXEL POSITION VALUES - Based on resolution
    x_img = (-y_lidar[indices]/res).astype(np.int32) # x axis is -y in LIDAR
    y_img = (x_lidar[indices]/res).astype(np.int32)  # y axis is -x in LIDAR
                                                     # will be inverted later
    x_img = (x_lidar[indices] / res).astype(np.int32)
    y_img = (-y_lidar[indices]/res).astype(np.int32)

    # SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
    # floor used to prevent issues with -ve vals rounding upwards
    x_img -= int(np.floor(side_range[0]/res))
    y_img -= int(np.floor(fwd_range[0]/res))

    # CLIP HEIGHT VALUES - to between min and max heights
    pixel_values = np.clip(a = z_lidar[indices],
                           a_min=min_height,
                           a_max=max_height)

    # RESCALE THE HEIGHT VALUES - to be between the range 0-255
    pixel_values  = scale_to_255(pixel_values,min_height,max_height)

    # FILL PIXEL VALUES IN IMAGE ARRAY
    x_max = int((side_range[1] - side_range[0])/res)
    y_max = int((fwd_range[1] - fwd_range[0])/res)
    im = np.zeros([y_max, x_max], dtype=np.uint8)
    im[-y_img, x_img] = pixel_values # -y because images start from top left

    # Convert from numpy array to a PIL image
    image = Image.fromarray(im)
    if saveto is not None:
        image.save(saveto)
    else:
     return im
def birds_eye_point_cloud(points,
                          side_range=(-10, 10),
                          fwd_range=(-10,10),
                          res=0.1,
                          min_height = -0.5,
                          max_height = 2,
                          saveto=None):


    x_lidar = points[:, 0]
    y_lidar = points[:, 1]
    z_lidar = points[:, 2]
    print(points.shape)
    # r_lidar = points[:, 3]  # Reflectance

    # INDICES FILTER - of values within the desired rectangle
    # Note left side is positive y axis in LIDAR coordinates
    ff = np.logical_and((x_lidar > fwd_range[0]), (x_lidar < fwd_range[1]))
    ss = np.logical_and((y_lidar > -side_range[1]), (y_lidar < -side_range[0]))
    indices = np.argwhere(np.logical_and(ff,ss)).flatten()

    # CONVERT TO PIXEL POSITION VALUES - Based on resolution
    x_img = (-y_lidar[indices]/res).astype(np.int32) # x axis is -y in LIDAR
    y_img = (x_lidar[indices]/res).astype(np.int32)  # y axis is -x in LIDAR
                                                     # will be inverted later


    # SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
    # floor used to prevent issues with -ve vals rounding upwards
    x_img -= int(np.floor(side_range[0]/res))
    y_img -= int(np.floor(fwd_range[0]/res))

    # CLIP HEIGHT VALUES - to between min and max heights
    pixel_values = np.clip(a = z_lidar[indices],
                           a_min=min_height,
                           a_max=max_height)

    # RESCALE THE HEIGHT VALUES - to be between the range 0-255
    pixel_values  = scale_to_255(pixel_values,min_height,max_height)

    # FILL PIXEL VALUES IN IMAGE ARRAY
    x_max = int((side_range[1] - side_range[0])/res)
    y_max = int((fwd_range[1] - fwd_range[0])/res)
    im = np.zeros([y_max, x_max], dtype=np.uint8)
    im[-y_img, x_img] = pixel_values # -y because images start from top left

    # Convert from numpy array to a PIL image
    image = Image.fromarray(im)
    if saveto is not None:
        image.save(saveto)
    else:
     return im
# define color pallet for bounding boxes
pallet = [
    (255, 0, 0),     # Blue
    (0, 255, 0),     # Green
    (0, 0, 255),     # Red
    (0, 255, 255),   # Yellow
    (255, 255, 0),   # Cyan
    (255, 0, 255),   # Magenta
    (255, 153, 204), # Purple
    (51, 153, 255),  # Orange
    (204, 153, 255), # Pink
]


# method to turn a number within a range to a color value (for point cloud reporjection)
def num_to_rgb(val, max_val=3):
    if (val > max_val):
        raise ValueError("val must not be greater than max_val")
    if (val < 0 or max_val < 0):
        raise ValueError("arguments may not be negative")
    
    i = (val / max_val)
    i = (-(i - 1) ** 4) + 1
    i *= 255
    r = round(math.sin(0.024 * i + 4) * 127 + 128)
    g = round(math.sin(0.024 * i + 2) * 127 + 128)
    b = round(math.sin(0.024 * i + 0) * 127 + 128)
    return (r,g,b)


# reproject lidar onto image



def repro_lidar(img, points, lidar2img):
    IMG_H,IMG_W,_ = img.shape
    points = points[:, 0:3]
    z=points[:,2]
    points = np.insert(points,3,1,axis=1).T
    print(z)
    newzs=[]
    for i in z:
        if (i > -0.5):
            newzs.append(i)
    lidar2img = lidar2img[:3, :4]

    cam = np.matrix(lidar2img) * points # Kitti style transform: P2 * R0_rect * Tr_points_to_cam * points
    cam = np.delete(cam,np.where(cam[2,:]<0)[1],axis=1)
    cam[:2] /= cam[2,:]

    # filter point out of canvas
    u,v,z = cam
    u_out = np.logical_or(u<0, u>IMG_W)
    v_out = np.logical_or(v<0, v>IMG_H)
    outlier =np.logical_or(u_out, v_out)
    cam = np.delete(cam,np.where(outlier),axis=1)

    # generate color map from depth
    us, vs, zs = np.array(cam)



    z_max = np.max(zs)


    z_min = np.min(zs)


    radius_max = math.ceil(IMG_W/1500)
    for u, v, z in zip(us, vs, zs):
        z_ratio = (z - z_min) / (z_max - z_min)
        z_ratio = (-(z_ratio - 1) ** 4) + 1
        point_radius = math.floor(radius_max - (z_ratio * radius_max)) + 1
        img = cv2.circle(img, (int(u), int(v)), radius=point_radius, color=num_to_rgb(z, z_max), thickness=-1)

    return img


# reproject bounding box onto image
def repro_box(img, box, lidar2img, color=(0,255,0)):
    IMG_H,IMG_W,_ = img.shape
    box = np.insert(box.T,3,1,axis=1)
    lidar2img = lidar2img[:3, :4]

    box = np.matrix(lidar2img) @ box
    box = np.delete(box,np.where(box[2,:]<0)[1],axis=1)
    box[:2] /= box[2,:]

    if box.size == 0:
        return img
    
    # generate color map from depth
    us, vs, zs = np.array(box)
    
    line_thickness = math.ceil(IMG_W/1500)
    try:
        img = cv2.line(img, (int(us[1]), int(vs[1])), (int(us[2]), int(vs[2])), color=color, thickness=line_thickness)
        img = cv2.line(img, (int(us[2]), int(vs[2])), (int(us[3]), int(vs[3])), color=color, thickness=line_thickness)
        img = cv2.line(img, (int(us[3]), int(vs[3])), (int(us[4]), int(vs[4])), color=color, thickness=line_thickness)
        img = cv2.line(img, (int(us[4]), int(vs[4])), (int(us[1]), int(vs[1])), color=color, thickness=line_thickness)
        
        img = cv2.line(img, (int(us[5]), int(vs[5])), (int(us[6]), int(vs[6])), color=color, thickness=line_thickness)
        img = cv2.line(img, (int(us[6]), int(vs[6])), (int(us[7]), int(vs[7])), color=color, thickness=line_thickness)
        img = cv2.line(img, (int(us[7]), int(vs[7])), (int(us[0]), int(vs[0])), color=color, thickness=line_thickness)
        img = cv2.line(img, (int(us[0]), int(vs[0])), (int(us[5]), int(vs[5])), color=color, thickness=line_thickness)
        
        img = cv2.line(img, (int(us[0]), int(vs[0])), (int(us[1]), int(vs[1])), color=color, thickness=line_thickness)
        img = cv2.line(img, (int(us[5]), int(vs[5])), (int(us[4]), int(vs[4])), color=color, thickness=line_thickness)
        img = cv2.line(img, (int(us[6]), int(vs[6])), (int(us[3]), int(vs[3])), color=color, thickness=line_thickness)
        img = cv2.line(img, (int(us[7]), int(vs[7])), (int(us[2]), int(vs[2])), color=color, thickness=line_thickness)
        
        front_x = [int(min(vs[1], vs[4])), int(max(vs[3], vs[2]))]
        front_y = [int(min(us[1], us[4])), int(max(us[3], us[2]))]
        box_front = img[front_x[0]:front_x[1], front_y[0]:front_y[1]]
        color_rect = np.ones(box_front.shape, dtype=np.float32) * 255
        color_rect = cv2.rectangle(color_rect, (0, 0), (color_rect.shape[1], color_rect.shape[0]) , color, thickness=-1)
        res = cv2.addWeighted(box_front, 0.75, color_rect, 0.25, 1.0)
        img[front_x[0]:front_x[1], front_y[0]:front_y[1]] = res
            
    except:
        pass
    
    # for i, (u, v, z) in enumerate(zip(us, vs, zs)):
    #     img = cv2.putText(img, str(i), (int(u), int(v)), 1, 1, (255, 255, 255), thickness=1)
        
    return img


# create calibration matrices for different view points
def create_calib(
    IMG_W,
    IMG_H,
    cam_trans_x = 20, # meters
    cam_trans_y = 0, # meters
    cam_trans_z = -8, # meters
    cam_rot_y = 20, # degrees
):
    focal_point_x = int(IMG_W/2)
    focal_point_y = int(IMG_W/2)
    intrinsics = np.array([
            [focal_point_x, 0, int(IMG_W/2), 0],
            [0, focal_point_y, int(IMG_H/2), 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    extrinsics = np.array([
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [1, 0, 0, -1],
            [0, 0, 0, 1]
        ])
        
    I = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    trans = np.array([
            [1, 0, 0, cam_trans_x],
            [0, 1, 0, cam_trans_y],
            [0, 0, 1, cam_trans_z],
            [0, 0, 0, 1]
        ])
    

    theta = math.radians(cam_rot_y)
    rot = np.array([
            [math.cos(theta), 0, -math.sin(theta), 0],
            [0, 1, 0, 0],
            [math.sin(theta), 0, math.cos(theta), 0],
            [0, 0, 0, 1]
        ])
    
    lidar2img = intrinsics @ extrinsics @ trans @ rot

    return lidar2img
