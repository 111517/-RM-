import cv2
import dectecte
cap=cv2.VideoCapture(1,cv2.CAP_DSHOW)
a=dectecte.detectapi(weights='myweight/weights/best.pt')
while True:
    rec,img = cap.read()
    blue, g, r = cv2.split(img)  # 分离通道，在opencv中图片的存储通道为BGR非RBG
    ret2, binary = cv2.threshold(blue, 220, 255, 0)
    Gaussian = cv2.GaussianBlur(binary, (5, 5), 0)  # 高斯滤波
    result,names =a.detect([img])
    cv2.imshow("detect",binary)
    img=result[0][0] #第一张图片的处理结果图片
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  # 获取视频的宽度
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  # 获取视频的高度
    fps = cap.get(cv2.CAP_PROP_FPS)  # 获取视频的帧率
    for cls,(x1,y1,x2,y2),conf in result[0][1]: #第一张图片的处理结果标签。
        ##print(cls,(x1+x2)//2-width/2,height/2-(y1+y2)//2,conf)
       ## print(width, height, fps)
        cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,0))
        #roi = img[y1:y1 + 100, x1:x1 + 100]  # 提取ROI区域cv2.imshow('ROI', roi)cv2.waitKey()
        #cv2.imshow("roi", roi)
        #cv2.putText(img,names[cls],(x1,y1-20),cv2.FONT_HERSHEY_DUPLEX,1.5,(255,0,0))
    cv2.imshow("vedio",img)
    if cv2.waitKey(1)==ord('q'):
        break
