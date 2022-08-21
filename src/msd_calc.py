#!/usr/bin/env python
import  rospy
import  sys
import  numpy as np
from    std_msgs.msg import String,Float64
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
from tf.transformations import euler_from_quaternion
import  time 
from    datetime import datetime

# def callback(sub_robot_0_path,sub_robot_1_path):

# 	global robot_0_path
# 	global robot_1_path
# 	robot_0_path=np.array(sub_robot_0_path.poses)
# 	robot_1_path=np.array(sub_robot_1_path.poses)

# 	# print(distance.euclidean(init_poses[0], robot_1_pose))


def callback(msg):
	coverage_percentage=msg.data
	lol=150.0
	if coverage_percentage>5 and flag==0:
		
		time25=datetime.now()-t0
		flag==1
	if coverage_percentage>10 and flag1==0:
		time50=datetime.now()-t0	
		flag1==1	
	if coverage_percentage>15 and flag2==0:
		time90=datetime.now()-t0
		print("lol")		
		msd(time25,time50,time90)
		

def msd(time25,time50,time90):
	# checks if the path topics are published
	while not  any('/robot_0/path' in subl for subl in rospy.get_published_topics()) :
		print("No robot_x/path topic published. Retrying.........")
	# kills the random walk nodes
	for i in range(robots):
		os.system("rosnode kill "+ "/robot_"+str(i)+"/rw_node")

	robot_0_path = rospy.wait_for_message('robot_0/path',Path)
	robot_1_path = rospy.wait_for_message('robot_1/path',Path)


	global dist
	global prev
	global msd
	global X
	global dist1
	global prev1
	global msd1
	global X1
	dist=0
	dist1=0
	msd=[0]
	msd1=[0]
	mean1=[0]
	X=[0]
	X1=[0]
	Z=[0]
	prevw=0
	prev=np.array([-2,0])
	prev1=np.array([-1,0])
	count=0
	for i in range(np.shape(robot_0_path.poses)[0]):
	
	# distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])>0.1

		if (distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[[robot_0_path.poses[i-1].pose.position.x,robot_0_path.poses[i-1].pose.position.y]])==0):
		 	if (distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])>0.1 ):

			 	dist=dist+math.pow(distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev]),2)
			# dist=dist+distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])


				msd=np.append(msd,dist)

				prev=[robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y]
				X=np.append(X,robot_0_path.poses[i].header.stamp.secs)
	Y=msd


	for i in range(np.shape(robot_1_path.poses)[0]):
		
		if (distance.euclidean([robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y],[[robot_1_path.poses[i-1].pose.position.x,robot_1_path.poses[i-1].pose.position.y]])==0 ):
			if (distance.euclidean([robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y],[prev1])>0.1 ):
				dist1=dist1+math.pow(distance.euclidean([robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y],[prev1]),2)
				
		# dist1=dist1+distance.euclidean([robot_1_path.poses[i].position.x,robot_1_path.poses[i].pose.position.y],[prev1])


				msd1=np.append(msd1,dist1)
				prev1=[robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y]
			# prevw=yaw
				
				X1=np.append(X1,robot_1_path.poses[i].header.stamp.secs)
	Y1=msd1

	# mean=(Y1+Y)/2
	# print(np.shape(X1))
	# print(np.shape(X))
	# Data=np.array([10,11,12])
	Data=np.array([[time25,time50,time90]])
	fig,ax=plt.subplots()
	
	plt.table(cellText=Data,colLabels=('25%','50%','90%'),rowLabels=["Coverage Time"],loc='bottom',bbox=[0.0,-1,1,0.3])
	ax.plot(X,Y)
	ax.plot(X1,Y1)
	plt.title('MSD Mean Squared Displacement')
	plt.xlabel('Time(s)')
	plt.ylabel('MSD')
	# ax.plot(Z,Y1)
	plt.subplots_adjust(left=0.2,bottom=0.5)
	fig.show()


	# np.append(particle1_msd,robot_0_pose)
	rospy.spin()




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
global flag
global flag1
global flag2
flag=0
flag1=0
flag2=0
global t0
global robots
robots=rospy.get_param("/swarm/robots")
init_poses=np.array([[-2,0],[-1,0],[0,0],[0,1],[0,-2],[-2,-0.5],[-1,-0.5]])
rospy.init_node('msd_calc', anonymous=True)
t0= datetime.now()
# rate = rospy.Rate(1)
sub = rospy.Subscriber("/coverage_percentage",Float64,callback)

rospy.spin()		# main()
		







	