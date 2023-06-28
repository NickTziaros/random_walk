#!/usr/bin/env python2
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path, OccupancyGrid

robot_paths = []
map_dim = None

def path_callback(msg):
    global robot_paths
    robot_paths.append(msg)

def map_callback(msg):
    global map_dim
    map_dim = (msg.info.width, msg.info.height)

def draw_heatmap():
    global robot_paths, map_dim

    plt.ion()
    fig, ax = plt.subplots()
    heatmap = np.zeros(map_dim)

    for path in robot_paths:
        x = [int(round(pose.pose.position.x)) for pose in path.poses]
        y = [int(round(pose.pose.position.y)) for pose in path.poses]
        heatmap[y, x] += 1

    im = ax.imshow(heatmap, cmap='hot', interpolation='nearest')
    plt.colorbar(im)

    while not rospy.is_shutdown():
        plt.pause(0.01)

if __name__ == '__main__':
    rospy.init_node('heatmap')
    rospy.Subscriber('/robot_0/path', Path, path_callback)
    rospy.Subscriber('/robot_2/path', Path, path_callback)
    rospy.Subscriber('/merged_map', OccupancyGrid, map_callback)

    while map_dim is None:
        rospy.sleep(0.1)

    draw_heatmap()

    rospy.spin()
