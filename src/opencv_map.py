#!/usr/bin/env python
import numpy as np 
import cv2


img = cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/ground_truth_1.pgm",0)
img5=	cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/try.pgm",0)
img2 = cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/try.pgm",0)
ground_truth_cropped= img[42:438,42:438]

merged_map_cropped=img2[42:438,42:438]


# print img[45]
# print img2[45]
counter=0.0
ground_truth_black_counter=0.0
black_counter=0.0


for i in range(ground_truth_cropped.shape[0]):
	for j in range(ground_truth_cropped.shape[1]):
		if ground_truth_cropped[i][j] == merged_map_cropped[i][j] :
			counter=counter+1
		if ground_truth_cropped[i][j] == 0:
			ground_truth_black_counter = ground_truth_black_counter+1

print counter/(merged_map_cropped.shape[0]*merged_map_cropped.shape[1])

for i in range(merged_map_cropped.shape[0]):
	for j in range(merged_map_cropped.shape[1]):
		if merged_map_cropped[i][j] == 0 :
			black_counter=black_counter+1

# print (black_counter)
# print(ground_truth_black_counter)


print (black_counter/ground_truth_black_counter)
# print merged_map_cropped
cv2.waitKey(0)