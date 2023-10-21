import cv2
camera = cv2.VideoCapture(1)
i = 1
while i < 50:
    _, frame = camera.read()
    cv2.imwrite("D:/images/"+str(i)+'.png', frame, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
    cv2.imshow('frame', frame)
    i += 1
    if cv2.waitKey(200) & 0xFF == 27: # 按ESC键退出
        break
cv2.destroyAllWindows()