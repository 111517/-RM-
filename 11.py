import numpy as np
import cv2
# glob ：返回所有匹配的文件路径列表
import glob
# EPS表示迭代次数达到最大次数时停止
# MAX_ITER表示角点位置变化的最小值已经达到最小时停止迭代
# 两个条件都要满足
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# 以z轴为 0 取平面上的点
# 6*8矩阵来储存角点
objp = np.zeros((6 * 13, 3), np.float32)
objp[:, :2] = np.mgrid[0:13, 0:6].T.reshape(-1, 2)
# objpoints 来储存世界坐标系下的三维点坐标
# imgpoints 来储存像素坐标系下的二维点坐标
objpoints = []
imgpoints = []
# 选出所有拍摄的图片
images = glob.glob('D:/images/*.png')
# 转为黑白图片，提高效率
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# 找到棋盘格上的角点(返回到像素坐标系)
# ret来判断是否找到了角点
ret, corners = cv2.findChessboardCorners(gray, (13, 6), None)
if ret == True:
    # 输入物点，（11，11）表示窗口大小，（-1，-1）表示忽略掉的细微结构
    # 这样会更精确
    objpoints.append(objp)
    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    imgpoints.append(corners2)

# 画出找到的点(corners2->img,即亚像素坐标系到图像坐标系)
img = cv2.drawChessboardCorners(img, (13, 6), corners2, ret)
cv2.namedWindow('img', 0)
cv2.imshow('img', img)
while cv2.waitKey(100) != 27:
    if cv2.getWindowProperty('img', cv2.WND_PROP_VISIBLE) <= 0:
        break
cv2.destroyAllWindows()

# 直接调用calibrateCamera即可完成求解矩阵等操作
# mrx：相机内参数
# dist：畸变参数
# rvecs：旋转向量
# tveces：平移向量
ret, mrx, dist, rvecs, tveces = cv2.calibrateCamera(objpoints, imgpoints, (8, 6),
                                                    None, None)
print(mrx)
print("*****")
print(dist)
print(rvecs)
print(tveces)