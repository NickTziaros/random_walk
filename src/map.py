#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String,Float64
from nav_msgs.msg import OccupancyGrid
import numpy as np

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

    
def compare(ground_truth):
    grid = OccupancyGrid()
    ground_truth_fill=np.full((500,500), 100)
    merged_map_fill=np.full((500,500), 100)
    counter=0.0
    ground_truth_map_2d=convert2_2D(ground_truth_map,ground_truth_height,ground_truth_width)
    merged_map_2d=convert2_2D(merged_map,merged_map_height,merged_map_width)
    ground_truth_cropped= ground_truth_map_2d[40:440,40:445]
    merged_map_cropped= merged_map_2d[40:440,40:445]

    # ground_truth_map_2d=convert2_2D(ground_truth_map)
    for i in range(480):
        for j in range(485):
            ground_truth_fill[i,j]=ground_truth_map_2d[i,j]
    for i in range(merged_map_height):
        for j in range(merged_map_width):
            merged_map_fill[i,j]=merged_map_2d[i,j]
    ground_truth_cropped=ground_truth_fill[40:440,40:445] 
    merged_map_cropped=merged_map_fill[40:440,40:445]
    merged_map_cropped_flat=merged_map_cropped.flatten() 
    ground_truth_cropped_flat=ground_truth_cropped.flatten()   
 
    grid.data=ground_truth_cropped_flat   
    grid.info.width=405
    # ground_truth_cropped.shape[0]
    grid.info.height=400
    # ground_truth_cropped.shape[1]
    grid.info.origin.position.x=ground_truth_mapix
    grid.info.origin.position.y=ground_truth_mapiy
    grid.info.resolution=0.05


    pub.publish(grid)

def main():


    rospy.init_node('compare_maps', anonymous=True)
    rospy.Subscriber("/merged_map", OccupancyGrid , callback)
    # rospy.Subscriber("/my_namespace/map", OccupancyGrid , callback)
    rospy.Subscriber("/ground_truth", OccupancyGrid , ground_truth_callback)
    # rospy.Subscriber("/my_namespace/map", OccupancyGrid , callback)
    global pub 
    pub = rospy.Publisher("/newmap1",OccupancyGrid, queue_size=10)
    rate = rospy.Rate(10)
    #rate.sleep()
    while not rospy.is_shutdown():
        rate.sleep()
        compare(ground_truth_map)
        # spin() simply keeps python from exiting until this node is stopped
 

if __name__ == '__main__':
    main()