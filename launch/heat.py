#!/usr/bin/env python2

import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid

# Define the callback function for the robot path topics
def path_callback(path_msg, robot_id):
    # Convert the robot path points to heat map indices
    x_indices = []
    y_indices = []
    for pose in path_msg.poses:
        x = int((pose.pose.position.x - map_origin_x) / map_resolution)
        y = int((pose.pose.position.y - map_origin_y) / map_resolution)
        x_indices.append(x)
        y_indices.append(y)
    
    # Increment the heat map values at the corresponding indices
    for i in range(len(x_indices)):
        x = x_indices[i]
        y = y_indices[i]
        if x >= 0 and x < map_width and y >= 0 and y < map_height:
            heat_map[y][x] += 1
    
    # Plot the heat map
    plt.figure(1)
    plt.clf()
    plt.imshow(heat_map, cmap='hot', interpolation='nearest', extent=(0, map_width*map_resolution, 0, map_height*map_resolution))
    plt.title(f"Heat map of robot")
    plt.pause(0.01)

# Define the callback function for the map topic
def map_callback(map_msg):
    # Get the map dimensions and resolution
    global map_width, map_height, map_resolution, map_origin_x, map_origin_y
    map_width = map_msg.info.width
    map_height = map_msg.info.height
    map_resolution = map_msg.info.resolution
    map_origin_x = map_msg.info.origin.position.x
    map_origin_y = map_msg.info.origin.position.y
    
    # Initialize the heat map array
    global heat_map
    heat_map = [[0] * map_width for _ in range(map_height)]
    
    # Subscribe to the robot path topics
    robot0_path_sub = rospy.Subscriber('/robot_0/path', Path, path_callback, callback_args=0)
    robot2_path_sub = rospy.Subscriber('/robot_2/path', Path, path_callback, callback_args=2)

# Define the main function
def main():
    # Initialize the ROS node
    rospy.init_node('robot_heat_map')
    
    # Subscribe to the map topic
    map_sub = rospy.Subscriber('/merged_map', OccupancyGrid, map_callback)
    
    # Set up the heat map figure
    plt.figure(1)
    plt.xticks([])
    plt.yticks([])
    plt.tight_layout()
    plt.draw()
    
    # Start the ROS node
    rospy.spin()

if __name__ == '__main__':
    main()
