#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

class OccupancyGridComparator:
    def __init__(self):
        self.map1 = None
        self.map2 = None
        self.width = None
        self.height = None
        self.publisher = rospy.Publisher("/coverage_percentage", Float32, queue_size=10)

        # Wait for map1 and map2 topics to be published
        rospy.loginfo("Waiting for map1 and map2 topics to be published...")
        while self.map1 is None or self.map2 is None:
            try:
                self.map1 = rospy.wait_for_message("map1_topic", OccupancyGrid, timeout=5.0)
                self.map2 = rospy.wait_for_message("map2_topic", OccupancyGrid, timeout=5.0)
            except rospy.exceptions.ROSException:
                rospy.logwarn("Map topics not yet published, retrying...")

        # Get map size
        self.width = self.map1.info.width
        self.height = self.map1.info.height

        # Calculate similarity
        similarity = self.calculate_similarity()

        # Publish result
        percentage = Float32()
        percentage.data = similarity
        self.publisher.publish(percentage)

        # Subscribe to map1 and map2 topics
        self.map1_subscriber = rospy.Subscriber("map1_topic", OccupancyGrid, self.map1_callback)
        self.map2_subscriber = rospy.Subscriber("map2_topic", OccupancyGrid, self.map2_callback)

    def map1_callback(self, msg):
        self.map1 = msg

        # Calculate similarity
        similarity = self.calculate_similarity()

        # Publish result
        percentage = Float32()
        percentage.data = similarity
        self.publisher.publish(percentage)

    def map2_callback(self, msg):
        self.map2 = msg

    def calculate_similarity(self):
        if self.map1 is None or self.map2 is None:
            return 0.0

        # Count number of matching cells
        num_matching_cells = 0
        for i in range(self.width * self.height):
            if self.map1.data[i] == self.map2.data[i]:
                num_matching_cells += 1

        # Calculate percentage of similarity
        similarity = float(num_matching_cells) / float(self.width * self.height)

        return similarity

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_comparator')
    occupancy_grid_comparator = OccupancyGridComparator()
    rospy.spin()
