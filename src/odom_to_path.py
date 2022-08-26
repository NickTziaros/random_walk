#!/usr/bin/env python
import rospy
import  numpy as np
import sys
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path = Path()

def odom_cb(data):
    global path
    path.header = data.header
    path.header.stamp=rospy.Time.now()
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    



def main():
    path_pub.publish(path)
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('path_node')
    args=rospy.myargv(argv=sys.argv)
    robotname= args[1]
    rate = rospy.Rate(0.6)
    odom_sub = rospy.Subscriber("/{}/odom".format(robotname), Odometry, odom_cb)
    path_pub = rospy.Publisher("/{}/path".format(robotname), Path, queue_size=10)
    while not rospy.is_shutdown():
        main()