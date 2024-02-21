import  cv2, numpy as np
img = cv2.imread('/home/gihong/Downloads/교육/05. Camera/Example Image.jpg',cv2.IMREAD_GRAYSCALE)
cv2.imshow("img", img)
print(img)
cv2.waitKey(0)