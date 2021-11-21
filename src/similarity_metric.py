#!/usr/bin/env python3
import numpy as np 
import cv2
import image_similarity_measures
from image_similarity_measures.quality_metrics import rmse, psnr


def get_dmap(ground_truth_cropped,merged_map_cropped):
	dmap = [[0 for x in range(ground_truth_cropped.shape[0])] for y in range(ground_truth_cropped.shape[1])]	
	for i in range(ground_truth_cropped.shape[0]):
		for j in range(ground_truth_cropped.shape[1]):
			if ground_truth_cropped[i][j] == 0 :
				dmap[i][j]=0
			else:
				dmap[i][j]=200000
	return dmap

# def mse(imageA, imageB):
# 	# the 'Mean Squared Error' between the two images is the
# 	# sum of the squared difference between the two images;
# 	# NOTE: the two images must have the same dimension
# 	err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
# 	err /= float(imageA.shape[0] * imageA.shape[1])
	
# 	# return the MSE, the lower the error, the more "similar"
# 	# the two images are
# 	return err	


ground_truth= cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/ground_truth_1.pgm")
merged_map= cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/try_1lol.pgm")


ground_truth_cropped= ground_truth[42:438,42:438]
merged_map_cropped= merged_map[42:438,42:438]
# cv2.resize(ground_truth_cropped, dim, interpolation = cv2.INTER_AREA)
# cv2.resize(merged_map_cropped, dim, interpolation = cv2.INTER_AREA)
# dmap=get_dmap(ground_truth_cropped,merged_map_cropped)
lol=rmse(ground_truth_cropped ,merged_map_cropped)
print(lol)
# print np.matrix(dmap)
# print np.matrix(ground_truth_cropped)
# print(np.shape(ground_truth_cropped))
# print(np.shape(merged_map_cropped))
# # cv2.imshow("Ground Truth",ground_truth_cropped)
# # cv2.imshow("Merged Map",merged_map_cropped)

# # print img[45]
# # print img2[45]
# counter=0.0

# for i in range(ground_truth_cropped.shape[0]):
# 	for j in range(ground_truth_cropped.shape[1]):
# 		if ground_truth_cropped[i][j] == merged_map_cropped[i][j] :
# 			counter=counter+1
# print counter/(merged_map_cropped.shape[0]*merged_map_cropped.shape[1])
# # print ground_truth_cropped
cv2.waitKey(0)









