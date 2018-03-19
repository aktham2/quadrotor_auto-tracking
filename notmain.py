from takeStill import *
import cv2

takeStill('test')

img1 = cv2.imread('pictures/test.jpg',0)
fast = cv2.FastFeatureDetector_create(threshold=25)

kp1 = fast.detect(img1,None)
img11 = cv2.drawKeypoints(img1,kp1,None,color=(0,0,255))

cv2.imshow('img',img11)
cv2.waitKey(0)
cv2.destroyAllWindows()
