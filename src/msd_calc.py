#!/usr/bin/env python
import  rospy
import  sys
import  numpy as np
import  std_msgs.msg  
from    geometry_msgs.msg import Twist,Point,Pose
from    nav_msgs.msg import Odometry,Path
from    robot_class import robot
from    laser_class import laser
import  math
from nav_msgs.msg import OccupancyGrid
import  time 
from    datetime import datetime 
import  os
import 	message_filters
from scipy.spatial import distance
import matplotlib.pyplot as plt

# def callback(sub_robot_0_path,sub_robot_1_path):

# 	global robot_0_path
# 	global robot_1_path
# 	robot_0_path=np.array(sub_robot_0_path.poses)
# 	robot_1_path=np.array(sub_robot_1_path.poses)

# 	# print(distance.euclidean(init_poses[0], robot_1_pose))


# def timer_callback(timer):
def main():
	robot_0_path = rospy.wait_for_message('robot_0/path',Path)
	robot_1_path = rospy.wait_for_message('robot_1/path',Path)
	# print(robot_0_path.poses)
	# global robot_0_path

	global dist
	global prev
	global msd
	global Y
	# global robot_1_path
	global dist1
	global prev1
	global msd1
	global Y1
	prev=np.array([-2,0])
	prev1=np.array([-1,0])
	# print(robot_0_path.poses[i].pose.position.x-prev[0])
	for i in range(np.shape(robot_0_path.poses)[0]):
		xd=prev[0]
		xd2=prev[1]

		# if (robot_0_path.poses[i].pose.position.x-xd>0.000) or (robot_0_path.poses[i].pose.position.y-xd2>0.000):
		# 	dist=dist+distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])
		dist=dist+distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])


		msd=np.append(msd,dist)
		prev=[robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y]
		Y=np.append(Y,robot_0_path.poses[i].header.seq)
	X=msd

	for i in range(np.shape(robot_1_path.poses)[0]):
		xd=prev1[0]
		xd2=prev1[1]

		# if (robot_0_path.poses[i].pose.position.x-xd>0.000) or (robot_0_path.poses[i].pose.position.y-xd2>0.000):
		# 	dist=dist+distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])
		dist1=dist1+distance.euclidean([robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y],[prev1])


		msd1=np.append(msd1,dist1)
		prev1=[robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y]
		Y1=np.append(Y1,robot_1_path.poses[i].header.seq)
	X1=msd1
	
	fig,ax=plt.subplots()
	ax.plot(X,Y)
	ax.plot(X1,Y1)
	fig.show()
	# np.append(particle1_msd,robot_0_pose)
	rospy.spin()



if __name__ == '__main__':
    try:
		global init_poses
		global particle1_msd
		global prev
		global clock
		global dist
		global robot_1_path
		global dist1
		global prev1
		global msd1
		global Y1
		dist=0
		dist1=0
		prev=np.array([-2,0])
		prev1=[-1,0]
		msd=[0]
		msd1=[0]
		Y=[0]
		Y1=[0]
		init_poses=np.array([[-2,0],[-1,0],[0,0],[0,1],[0,-2],[-2,-0.5],[-1,-0.5]])
		rospy.init_node('msd_calc', anonymous=True)
		rate = rospy.Rate(1)

		# timer = rospy.Timer(rospy.Duration(10), timer_callback)
		# sub_robot_12 = message_filters.Subscriber('ground_truth',OccupancyGrid)
		sub_robot_0_path = rospy.wait_for_message('robot_0/path',Path)
		# sub_robot_0_path = message_filters.Subscriber('robot_0/path',Path)
		# sub_robot_1_path = message_filters.Subscriber('robot_1/path',Path)
		# sub_robot_2_path = message_filters.Subscriber('robot_2/path',Path)
		# sub_robot_3_path = message_filters.Subscriber('robot_3/path',Path)
		# sub_robot_4_path = message_filters.Subscriber('robot_4/path',Path)
		# sub_robot_5_path = message_filters.Subscriber('robot_5/path',Path)


		# ts = message_filters.TimeSynchronizer([sub_robot_0_path,sub_robot_1_path], 1)
		# ts.registerCallback(callback)
		# while not rospy.is_shutdown():
		main()
		







    except rospy.ROSInterruptException:
        pass