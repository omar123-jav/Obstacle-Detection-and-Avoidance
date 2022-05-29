import cv2
import numpy as np
img=cv2.imread('output/vis_points.png')
kernel = np.ones((3, 3), np.uint8)

# Using cv2.erode() method
image = cv2.erode(img, kernel,iterations=1)

# Displaying the image
cv2.imwrite('output/morgh.png',image)
