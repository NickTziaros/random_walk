#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String,Float64
from nav_msgs.msg import OccupancyGrid
import numpy as np
import image_similarity_measures
from image_similarity_measures.quality_metrics import rmse, psnr ,ssim,fsim
import cv2


def callback(msg):
    global merged_map
    global merged_map_height
    global merged_map_width
    merged_map=msg.data
    merged_map_height=msg.info.height
    merged_map_width=msg.info.width
def ground_truth_callback(msg):
    global ground_truth_map
    global ground_truth_mapiy
    global ground_truth_mapix
    global ground_truth_width
    global ground_truth_height
    ground_truth_map=msg.data
    ground_truth_mapix=msg.info.origin.position.x
    ground_truth_mapiy=msg.info.origin.position.y
    ground_truth_width=msg.info.width
    ground_truth_height=msg.info.height
def convert2_2D(arr_1d,height,width):
    
    arr_2d = np.reshape( arr_1d, (height,width ))

    return arr_2d

    
def compare():

    ground_truth_fill=np.full((500,500), 100)
    merged_map_fill=np.full((500,500), 100)
    grid = OccupancyGrid()
    # initializes the counter for the percentage formula.
    counter=0.0
    counter2=0.0
    
    # converts the ground_truth and merged_map maps into 2d lists.
    ground_truth_map_2d=convert2_2D(ground_truth_map,ground_truth_height,ground_truth_width)
    merged_map_2d=convert2_2D(merged_map,merged_map_height,merged_map_width)

    # # crops the 2d version of the ground_truth and merged_map maps
    # ground_truth_cropped= ground_truth_map_2d[40:440,40:445]
    # merged_map_cropped= merged_map_2d[40:440,40:445]

    # fills the two 500x500 lists with the croped maps.
    # ground_truth_fill=np.zeros((ground_truth_height, ground_truth_width))
    ground_truth_fill=ground_truth_fill.astype(Float64)
    for i in range(ground_truth_height):
        for j in range(ground_truth_width):
            if (ground_truth_map_2d[i,j] == -1) :
                ground_truth_fill[i,j]=0.5
            elif (ground_truth_map_2d[i,j] == 100):
                ground_truth_fill[i,j]=1
            else:
                ground_truth_fill[i,j]=ground_truth_map_2d[i,j]
            # print(ground_truth_fill[i,j])   


    # merged_map_fill=np.zeros((merged_map_height, merged_map_width))
    merged_map_fill=merged_map_fill.astype(Float64)
    for i in range(merged_map_height):
        for j in range(merged_map_width):
            merged_map_fill[i,j]=merged_map_2d[i,j]
            if (merged_map_2d[i,j] == -1) :
                merged_map_fill[i,j] = 0.5
            elif (merged_map_2d[i,j]== 100):
                merged_map_fill[i,j]=1
            else:
                merged_map_fill[i,j]=merged_map_2d[i,j]             

    # crops the two lists filled with the maps
    ground_truth_cropped=ground_truth_fill[40:440,40:445]

    merged_map_cropped=merged_map_fill[40:440,40:445]

    print(merged_map_cropped.shape[0]*merged_map_cropped.shape[1])
    counter=0.0
    counter2=0.0
    for i in range(merged_map_cropped.shape[0]):
        for j in range(merged_map_cropped.shape[1]):
            if (merged_map_cropped[i,j]==1) and (ground_truth_cropped[i,j]==1):
                counter=counter+1
            if (ground_truth_cropped[i,j]==1):
                counter2=counter2+1

    score=counter/counter2
    print(score)  







    # print(merged_map_cropped.tolist())
    # print(type(merged_map_cropped))
    # print(ground_truth_cropped.shape)
    # cv2.imshow("window_name2", ground_truth_cropped)
    # cv2.waitKey(0) 
    # cv2.destroyAllWindows() 





def main():


    rospy.init_node('compare_maps1', anonymous=True)
    rospy.Subscriber("/merged_map", OccupancyGrid , callback)
    rospy.Subscriber("/ground_truth", OccupancyGrid , ground_truth_callback)
    # rospy.Subscriber("/my_namespace/map", OccupancyGrid , callback)
    global pub 
    pub = rospy.Publisher("/map2",OccupancyGrid, queue_size=10)
    rate = rospy.Rate(10)
    #rate.sleep()
    while not rospy.is_shutdown():
        rate.sleep()
        compare()
        # spin() simply keeps python from exiting until this node is stopped
 

if __name__ == '__main__':
    main()