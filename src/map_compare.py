#!/usr/bin/env python2
# license removed for brevity
import rospy
from std_msgs.msg import String,Float64
from nav_msgs.msg import OccupancyGrid
import numpy as np
import  time 

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
    # creates the 2d lists that will contain the 2d maps and fills the with the value 100.
    ground_truth_fill=np.full((500,500), 100)
    merged_map_fill=np.full((500,500), 100)
    
    # initializes the counter for the percentage formula.
    counter=0.0
    counter2=0.0
    OCRcounter=0.0
    FCRcounter=0.0
    FCRcounter2=0.0
    OCRcounter2=0.0
    # converts the ground_truth and merged_map maps into 2d lists.
    ground_truth_map_2d=convert2_2D(ground_truth_map,ground_truth_height,ground_truth_width)
    merged_map_2d=convert2_2D(merged_map,merged_map_height,merged_map_width)

    # # crops the 2d version of the ground_truth and merged_map maps
    # ground_truth_cropped= ground_truth_map_2d[40:440,40:445]
    # merged_map_cropped= merged_map_2d[40:440,40:445]

    # fills the two 500x500 lists with the croped maps.
    for i in range(ground_truth_width-1):
        for j in range(ground_truth_height):
            ground_truth_fill[i,j]=ground_truth_map_2d[i,j]
    for i in range(merged_map_height):
        for j in range(merged_map_width):
            merged_map_fill[i,j]=merged_map_2d[i,j]

    # crops the two lists filled with the maps
    ground_truth_cropped=ground_truth_fill[40:440,40:441] 
    merged_map_cropped=merged_map_fill[40:440,40:441]

    #Reverts the 2d lists into 1d so they can be published for debuging. 
    merged_map_cropped_flat=merged_map_cropped.flatten() 
    ground_truth_cropped_flat=ground_truth_cropped.flatten()     
    # Filles the Occupancy grid map msg 
    # grid.data=ground_truth_cropped_flat
    # grid.info.height=ground_truth_cropped.shape[0]
    # grid.info.width=ground_truth_cropped.shape[1]
    # grid.info.origin.position.x=ground_truth_mapix
    # grid.info.origin.position.y=ground_truth_mapiy
    # grid.info.resolution=0.05

# Calculates the percentage of the known cells in the map   
    for i in range(len(ground_truth_cropped_flat)):
        
            if merged_map_cropped_flat[i]>-1  :
                counter=counter+1
    percent= (counter/(400*401))*100

# Calculates the percentage of identical cells in merged_map and ground_truth map   
    for i in range(len(ground_truth_cropped_flat)):
        
            if merged_map_cropped_flat[i]==ground_truth_cropped_flat[i]  :
                counter2=counter2+1
    percent2= (counter2/(400*401))*100

    # print(percent)
    # print(counter2)
    pub.publish(percent)

    for i in range(len(ground_truth_cropped_flat)):
        
            if merged_map_cropped_flat[i]==100 and ground_truth_cropped_flat[i]==100  :
                OCRcounter=OCRcounter+1
            if ground_truth_cropped_flat[i]==100  :
                OCRcounter2=OCRcounter2+1

    for i in range(len(ground_truth_cropped_flat)):
        
            if merged_map_cropped_flat[i]==0 and ground_truth_cropped_flat[i]==0  :
                FCRcounter=FCRcounter+1
            if ground_truth_cropped_flat[i]==0  :
                FCRcounter2=FCRcounter2+1

    OCRpercent= (OCRcounter/OCRcounter2)
    FCRpercent= (FCRcounter/FCRcounter2)
    pub1.publish(OCRpercent)
    pub2.publish(FCRpercent)












def main():
    rospy.init_node('compare_maps', anonymous=True)
    topic_depend_1 = '/merged_map'
    topic_depend_2  = '/ground_truth'
    message_type = OccupancyGrid
    retry_interval=1
 
    while not rospy.is_shutdown():

        try:
            rospy.loginfo('Waiting for publisher for topic %s...',topic_depend_2)
            rospy.wait_for_message(topic_depend_2, message_type, timeout=1)
            rospy.loginfo('Got message for topic %s...',topic_depend_2)
            break
        except rospy.ROSException:
            rospy.logwarn("No publisher available for topic %s. Retrying in %s seconds...",topic_depend_2,retry_interval)
            time.sleep(retry_interval)

    while not rospy.is_shutdown():

        try:
            rospy.loginfo('Waiting for publisher for topic %s...',topic_depend_1)
            rospy.wait_for_message(topic_depend_1, message_type, timeout=1)
            rospy.loginfo('Got message for topic %s...',topic_depend_1)
            break
        except rospy.ROSException:
            rospy.logwarn("No publisher available for topic %s. Retrying in %s seconds...",topic_depend_1,retry_interval)
            time.sleep(retry_interval)

    
    rospy.Subscriber("/merged_map", OccupancyGrid , callback)
    rospy.Subscriber("/ground_truth", OccupancyGrid , ground_truth_callback)


    
     
    global pub 
    global pub1
    global pub2
    global pub3

    pub = rospy.Publisher("/coverage_percentage",Float64, queue_size=10)
    pub1 = rospy.Publisher("/OCR",Float64, queue_size=10)
    pub2 = rospy.Publisher("/FCR",Float64, queue_size=10)
    pub3 = rospy.Publisher("/croped_map",OccupancyGrid, queue_size=10)

    rate = rospy.Rate(10)
    #rate.sleep()
    while not rospy.is_shutdown():
        rate.sleep()
        compare()
        # spin() simply keeps python from exiting until this node is stopped
 

if __name__ == '__main__':
    main()