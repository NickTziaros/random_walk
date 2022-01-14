#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String,Float64
from nav_msgs.msg import OccupancyGrid
import numpy as np


def callback(msg):
    global ogm
    ogm=msg.data
    
def ground_truth_callback(msg):
    global ground_truth_map
    ground_truth_map=msg.data

def convert2_2D(arr_1d):
    
    arr_2d = np.reshape( arr_1d, (485, 480))
    return arr_2d

    
def compare(ground_truth,current_map):
    counter=0.0
    counter2=0.0
    ground_truth_map_2d=convert2_2D(ground_truth_map)
    ground_truth_cropped= ground_truth_map_2d[42:438,42:438]

    current_map_2d=convert2_2D(current_map)
    current_map_cropped= current_map_2d[42:438,42:438]


    for i in range(np.shape(ground_truth_cropped)[0]):
        for j in range(np.shape(ground_truth_cropped)[1]):
            if current_map_cropped[i][j]>-1  :
                counter=counter+1
            if  ground_truth_cropped[i][j]==-1 :
                counter2=counter2+1

    percent= (counter/(np.shape(ground_truth_cropped)[0]*np.shape(ground_truth_cropped)[1]-counter2))*100
    pub.publish(percent)


def main():


    rospy.init_node('compare_maps', anonymous=True)
    rospy.Subscriber("/merged_map", OccupancyGrid , callback)
    rospy.Subscriber("/ground_truth", OccupancyGrid , ground_truth_callback)
    global pub 
    pub = rospy.Publisher("coverage_percentage",Float64, queue_size=10)
    rate = rospy.Rate(10)
    rate.sleep()
    while not rospy.is_shutdown():
        rate.sleep()
        compare(ground_truth_map,ogm)
        # spin() simply keeps python from exiting until this node is stopped
 

if __name__ == '__main__':
    main()