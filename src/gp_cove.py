#!/usr/bin/env python2

import rospy
from nav_msgs.msg import OccupancyGrid

def crop_and_resize(map_msg):
    width = map_msg.info.width
    height = map_msg.info.height

    if height < 2:
        rospy.logerr("Map is too small to crop the last row")
        return None

    # Crop last row
    cropped_data = map_msg.data[:width * (height - 1)]
    cropped_info = map_msg.info
    cropped_info.height = height - 1

    # Resize to 480x480
    new_width = 480
    new_height = 480
    step_x = width // new_width
    step_y = height // new_height

    new_data = []
    for y in range(new_height):
        for x in range(new_width):
            index = x * step_x + y * step_y * width
            new_data.append(cropped_data[index])

    cropped_info.width = new_width
    cropped_info.height = new_height

    cropped_map = OccupancyGrid()
    cropped_map.header = map_msg.header
    cropped_map.info = cropped_info
    cropped_map.data = new_data

    return cropped_map

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_cropper')

    # Subscribe to map topic
    map_sub = rospy.Subscriber('/merged_map', OccupancyGrid, crop_and_resize)

    # Publish new map to topic
    new_map_pub =
