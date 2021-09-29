#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid


def callback(msg):
    global ogm
    ogm=msg.data
    
def ground_truth_callback(msg):
    global ground_truth_map
    ground_truth_map=msg.data
    
def compare(ground_truth,current_map):
    counter=0
    for i in range(len(ground_truth)):
        if current_map[i]==ground_truth[i]:
            # print current_map[i]
            counter=counter+1
    print counter
    print len(ground_truth)
    print len(current_map)
    percent=float(counter)/230400.0
    print(str(percent*100)+"%")
    # print "{:.2f}".format(percent)
    # if ground_truth[122111]==current_map[122111]:
    #     print('lol')
    # else:
    #     print('xd')


def main():


    rospy.init_node('map_compare', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid , callback)
    rospy.Subscriber("/ground_truth", OccupancyGrid , ground_truth_callback)
    rate = rospy.Rate(10)
    rate.sleep()
    compare(ground_truth_map,ogm)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()