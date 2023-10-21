import cv2
import numpy as np
import math
obj_points=[[-4.2,3.7,0],[4.2,3.7,0],[-4.2,-3.7,0],[4.2,-3.7,0]]
mtx =[[ 4.52592997e+03 , 0.00000000e+00 , 3.02083849e+02],
 [ 0.00000000e+00  ,4.21147920e+03, -6.99078517e+01],
 [ 0.00000000e+00 , 0.00000000e+00, 1.00000000e+00]]
mtx = np.array(mtx)
dist = [[-6.72576871e+00, -3.32306997e+00,  4.84083522e-01, -1.50676089e-03,-4.71233533e+01]]
dist = np.array(dist)
# 内参数矩阵
Camera_intrinsic = {"mtx": mtx, "dist": dist, }
img_points =[[287,126],[389,126],[287,298],[389,298]]  # 存储2D点
img_points = np.array(img_points,dtype=np.float64)
obj_points=np.array(obj_points,dtype=np.float64)
_, rvec, tvec = cv2.solvePnP(obj_points, img_points, Camera_intrinsic["mtx"], Camera_intrinsic["dist"])  # 解算位姿
distance = math.sqrt(tvec[0] ** 2 + tvec[1] ** 2 + tvec[2] ** 2)  # 计算距离
rvec_matrix = cv2.Rodrigues(rvec)[0]  # 旋转向量->旋转矩阵
print("旋转矩阵")
print(rvec_matrix)
print("tvec")
print(tvec)
rt =np.array([[rvec_matrix[0][0],rvec_matrix[0][1],tvec[0]],
              [rvec_matrix[1][0],rvec_matrix[1][1],tvec[1]],
              [rvec_matrix[2][0],rvec_matrix[2][1],tvec[2]]], dtype=np.float)
rt_i=np.linalg.inv(rt)
pi_i = np.linalg.inv(Camera_intrinsic["mtx"])
uv = np.array([[434,396,362,331,338,376,480],
               [349,331,315,300,347,368,371],
               [1,1,1,1,1,1,1]])
xy1 = 0.340909*rt_i.dot(pi_i.dot(uv))
print("xy1")
print(xy1)
proj_matrix = np.hstack((rvec_matrix, tvec))  # hstack: 水平合并
eulerAngles = cv2.decomposeProjectionMatrix(proj_matrix)[6]  # 欧拉角
pitch, yaw, roll = eulerAngles[0], eulerAngles[1], eulerAngles[2]
print(pitch,yaw)