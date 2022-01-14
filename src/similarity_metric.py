#!/usr/bin/env python3
import numpy as np 
import cv2
import image_similarity_measures
from image_similarity_measures.quality_metrics import rmse, psnr ,ssim,fsim




ground_truth= cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/ground_truth_1.pgm")
merged_map= cv2.imread("/home/nikos/catkin_ws/src/random_walk/stage_results/try_1lol.pgm")


ground_truth_cropped= ground_truth[42:438,42:438]
merged_map_cropped= merged_map[42:438,42:438]




rms_error=rmse(ground_truth_cropped ,merged_map_cropped)
ssim=ssim(ground_truth_cropped ,merged_map_cropped)
fsim=fsim(ground_truth_cropped ,merged_map_cropped)

print("Root mean square error: " + str(rms_error))
print("Structural Similarity Index: " + str(ssim))
print("Feature Similarity Index: " +str(fsim))

cv2.waitKey(0)









