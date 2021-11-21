#!/usr/bin/env python
import numpy as np 
import cv2


img = cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/ground_truth_1.pgm",0)
img5=	cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/try_1lol.pgm",0)
img2 = cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/try_1lol.pgm",0)
img3= img[42:438,42:438]
img6= img[42:438,42:438]
img4=img2[42:438,42:438]
cv2.imshow("croped",img3)
cv2.imshow("croped2",img4)

# print img[45]
# print img2[45]
counter=0.0

for i in range(img3.shape[0]):
	for j in range(img3.shape[1]):
		if img3[i][j] == img4[i][j] :
			counter=counter+1
print counter/(img4.shape[0]*img4.shape[1])

cv2.waitKey(0)