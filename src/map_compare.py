#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

def callback(msg):

   ogm=msg.data
   print(len(ogm))
    


    
def main():


    rospy.init_node('map_compare', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid , callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()