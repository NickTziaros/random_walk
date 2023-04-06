#!/usr/bin/env python2

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path

class Heatmap:

    def __init__(self):
        rospy.init_node('heatmap_node')

        # Define subscribers
        rospy.Subscriber('/path1', Path, self.path1_callback)
        rospy.Subscriber('/path2', Path, self.path2_callback)

        # Initialize data structures
        self.visits = None
        self.x_edges = None
        self.y_edges = None

        rospy.spin()

    def path1_callback(self, path):
        self.calculate_heatmap(path)

    def path2_callback(self, path):
        self.calculate_heatmap(path)

    def calculate_heatmap(self, path):
        # Calculate visits
        x = np.array([pose.pose.position.x for pose in path.poses])
        y = np.array([pose.pose.position.y for pose in path.poses])
        self.visits, self.x_edges, self.y_edges = np.histogram2d(x, y, bins=50)

        # Generate heatmap plot
        fig, ax = plt.subplots()
        im = ax.imshow(self.visits.T, extent=[self.x_edges[0], self.x_edges[-1], self.y_edges[0], self.y_edges[-1]], cmap='viridis')
        ax.set_title('Visits Heatmap')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        fig.colorbar(im)
        plt.show()

if __name__ == '__main__':
    hm = Heatmap()
