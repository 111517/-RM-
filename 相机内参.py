import cv2 as cv
import numpy as np
import glob
# 棋盘格内角点
w = 6
h = 13
# 每个棋盘小格的实际长度，单位为mm
chesslength = 330
# 计算亚像素角点时终止迭代阈值,最大计算次数30次，最大误差容限0.001
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# 准备格式如 (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)的3d角点
objp = np.zeros((w*h, 3), np.float32)
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 存储3D角点
imgpoints = [] # 存储2D角点

images = glob.glob('D:/images/*.png')
for fname in images:
    img = cv.imread(fname)  # 读取图片的格式为[h,w,c]
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # 计算棋盘格角点
    ret, corners = cv.findChessboardCorners(gray, (w, h), None)
    print(ret)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp * chesslength)
        # 角点精细化，其中corners为初始计算的角点向量
        corners = cv.cornerSubPix(gray, corners, (11, 11), (-1,-1), criteria)
        imgpoints.append(corners)
        # 绘制角点并展示
        cv.drawChessboardCorners(img, (w,h), corners, ret)
        cv.imshow('img', img)
        cv.waitKey(0)
cv.destroyAllWindows()

# 相机标定，依次返回标定结果、内置参数矩阵、畸变参数、旋转向量、平移向量
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints,(img.shape[1],img.shape[0]), None, None)
print("ret:", ret)        # ret为bool值
print("mtx:\n", mtx)      # 内参数矩阵
print("dist:\n", dist)    # 畸变系数 distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
print("rvecs:\n", rvecs)  # 旋转向量，外参数
print("tvecs:\n", tvecs)  # 平移向量，外参数

# 计算重投影误差
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print("total error: {}".format(mean_error/len(objpoints)))
