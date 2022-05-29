import cv2
import numpy as np
import glob
from tqdm import tqdm
import PIL.ExifTags
import PIL.Image


from matplotlib import pyplot as plt


# =====================================
# Function declarations
# =====================================
import numpy as np
import open3d as o3d

# Load binary point cloud
bin_pcd = np.fromfile("data/bin.bin", dtype=np.float32)

# Reshape and drop reflection values
points = bin_pcd.reshape((-1, 4))[:, 0:3]

# Convert to Open3D point cloud
o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

# Save to whatever format you like
o3d.io.write_point_cloud("output/pointcloud.png", o3d_pcd)