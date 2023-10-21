import cv2 as cv
import numpy as np

objp = np.zeros((2*2, 3), np.float32)
objp[0][0]=-6.8
objp[0][1]=3
objp[1][0]=6.8
objp[1][1]=3
objp[2][0]=-6.8
objp[2][1]=-3
objp[3][0]=6.8
objp[3][1]=-3
# Arrays to store object points and image points from all the images.
objpoints = [] # 存储3D角点
imgpoints = [] # 存储2D角点
print(objp)
