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
def callbackFCR(msg):
	global FCR
	FCR=msg.data
	# print(FCR)

def callbackOCR(msg):
	global OCR
	OCR=msg.data
	# print(OCR)

def callback(msg):
	# global coverage_percentage
	# global coverage_percentagetime
	coverage_percentage.append(msg.data)
	coverage_percentagetime.append(rospy.get_rostime().secs)	
	if msg.data>15 :
		
		msd()
		
# time25,time50,time90
def msd():
	# checks if the path topics are published
	while not  any('/robot_0/path' in subl for subl in rospy.get_published_topics()) :
		print("No robot_x/path topic published. Retrying.........")
	# robot_path=np.array((0,0,0,0,0,0))
	# for i in range(robots):
	# 	robot_path[i] = rospy.wait_for_message("/robot_{}/path".format(i),Path)
	# 	print("lol")
	robot_path=[0,0,0,0,0,0]
	robot_path[0]=rospy.wait_for_message("/robot_0/path",Path)
	robot_path[1]=rospy.wait_for_message("/robot_1/path",Path)
	robot_path[2]=rospy.wait_for_message("/robot_2/path",Path)
	robot_path[3]=rospy.wait_for_message("/robot_3/path",Path)
	robot_path[4]=rospy.wait_for_message("/robot_4/path",Path)
	robot_path[5]=rospy.wait_for_message("/robot_5/path",Path)


	msd=[[0],[0],[0],[0],[0],[0]]
	prev=[[-2,0],[-1,0],[0,0],[1,0],[-2,-0.5],[-1,-0.5]]	# dist=np.zeros((robots))
	# global dist	# global prev
	# global msd
	# global X
	# global dist1
	# global prev1
	# global msd1
	# global X1
	# dist=0
	# dist1=0
	# msd=[0]
	# msd1=[0]
	# mean1=[0]
	# X=[0]
	# X1=[0]
	# Z=[0]
	# prevw=0
	# # prev=np.array([-2,0])
	# # prev1=np.array([-1,0])
	# count=0
	# for i in range(np.shape(robot_0_path.poses)[0]):
	
	# distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])>0.1

	# 	if (distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[[robot_0_path.poses[i-1].pose.position.x,robot_0_path.poses[i-1].pose.position.y]])==0):
	# 	 	if (distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])>0.1 ):

	# 		 	dist=dist+math.pow(distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev]),2)
	# 		# dist=dist+distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])


	# 			msd=np.append(msd,dist)

	# 			prev=[robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y]
	# 			X=np.append(X,robot_0_path.poses[i].header.stamp.secs)
	# Y=msd
	# 	if not (distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[[robot_0_path.poses[i-1].pose.position.x,robot_0_path.poses[i-1].pose.position.y]])==0 ):


	# 		dist=dist+math.pow(distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev]),2)
	# 		# dist=dist+distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])


	# 		msd=np.append(msd,dist)

	# 		prev=[robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y]
	# 		X=np.append(X,robot_0_path.poses[i].header.stamp.secs)
	# Y=msd

	# for i in range(np.shape(robot_1_path.poses)[0]):
		
	# # 	if (distance.euclidean([robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y],[[robot_1_path.poses[i-1].pose.position.x,robot_1_path.poses[i-1].pose.position.y]])==0 ):
	# # 		if (distance.euclidean([robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y],[prev1])>0.1 ):
	# # 			dist1=dist1+math.pow(distance.euclidean([robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y],[prev1]),2)
				
	# # 	# dist1=dist1+distance.euclidean([robot_1_path.poses[i].position.x,robot_1_path.poses[i].pose.position.y],[prev1])


	# # 			msd1=np.append(msd1,dist1)
	# # 			prev1=[robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y]
	# # 		# prevw=yaw
				
	# # 			X1=np.append(X1,robot_1_path.poses[i].header.stamp.secs)
	# # for i in range(np.shape(robot_1_path.poses)[0]):
		
		
	# 	if not (distance.euclidean([robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y],[[robot_1_path.poses[i-1].pose.position.x,robot_1_path.poses[i-1].pose.position.y]])==0 ):

	# 		dist1=dist1+math.pow(distance.euclidean([robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y],[prev1]),2)
				
	# 	# dist1=dist1+distance.euclidean([robot_1_path.poses[i].position.x,robot_1_path.poses[i].pose.position.y],[prev1])


	# 		msd1=np.append(msd1,dist1)
	# 		prev1=[robot_1_path.poses[i].pose.position.x,robot_1_path.poses[i].pose.position.y]
	# 		# prevw=yaw
				
	# 		X1=np.append(X1,robot_1_path.poses[i].header.stamp.secs)
	# Y1=msd1





	Y=[[0],[0],[0],[0],[0],[0]]

	X=[[0],[0],[0],[0],[0],[0]]
	# print(X)
	for j in range(robots):
		# print(np.shape(robot_path[j].poses)[0])
		
		dist=0
		# dist[j]=np.zeros(((np.shape(robot_path[j].poses))[0]))	
		for i in range(np.shape(robot_path[j].poses)[0]):
			if not (distance.euclidean([robot_path[j].poses[i].pose.position.x,robot_path[j].poses[i].pose.position.y],[[robot_path[j].poses[i-1].pose.position.x,robot_path[j].poses[i-1].pose.position.y]])==0 ):
				if not (robot_path[j].poses[i].header.stamp.secs==robot_path[j].poses[i-1].header.stamp.secs):

					dist=dist+math.pow(distance.euclidean([robot_path[j].poses[i].pose.position.x,robot_path[j].poses[i].pose.position.y],prev[j]),2)
				# dist=dist+distance.euclidean([robot_0_path.poses[i].pose.position.x,robot_0_path.poses[i].pose.position.y],[prev])


					msd[j].append(dist)

					prev[j]=[robot_path[j].poses[i].pose.position.x,robot_path[j].poses[i].pose.position.y]
					# print(robot_path[j].poses[i].header.stamp.secs)
					X[j].append(robot_path[j].poses[i].header.stamp.secs)
					Y[j]=msd[j]		
		# print(len(Y[j]))			





























	# mean=(Y1+Y)/2
	# print(np.shape(X1))
	# print(np.shape(X))
	# Data=np.array([10,11,12])
	# Data=np.array([[time25,time50,time90]])
	fig,ax=plt.subplots(2,1, figsize=(8, 6))
	arr=np.array(X)

	# xd=X[0]+X[1]+X[2]+X[3]+X[4]+X[5]
	# for i in range(len(xd)):
	# 	xd[i]=xd[i]/robots
	ratio=np.array([[round(FCR,3),round(OCR,3)]])
	# plt.table(cellText=Data,colLabels=('25%','50%','95%'),rowLabels=["Coverage Time"],loc='bottom',bbox=[0.0,-1,1,0.3])
	plt.table(cellText=ratio,colLabels=('FCR','OCR'),loc='bottom',bbox=[0.0,-0.5,1,0.3])
	print(coverage_percentagetime)
	ax[1].plot(coverage_percentagetime,coverage_percentage)
	ax[1].set_title("Coverage")
	ax[0].set_xlabel("Time(s)")
	ax[0].set_xlabel("coverage Percentage")
	ax[0].set_title("MSD Mean Squared Displacement")
	ax[0].set_xlabel("Time(s)")
	ax[0].set_xlabel("MSD")
	ax[0].plot(X[1],Y[1])
	ax[0].plot(X[0],Y[0])
	ax[0].plot(X[2],Y[2])
	ax[0].plot(X[3],Y[3])	
	ax[0].plot(X[4],Y[4])
	ax[0].plot(X[5],Y[5])
	plt.tight_layout()
	# ax.plot(xd,Y[0])
	plt.subplots_adjust(bottom=0.2)
	# ax.plot(Z,Y1)
	# plt.subplots_adjust(left=0.2,bottom=0.5)
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
global coverage_percentage
global coverage_percentagetime
coverage_percentagetime=[]
coverage_percentage=[]
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
subFCR = rospy.Subscriber("/FCR",Float64,callbackFCR)
subOCR = rospy.Subscriber("/OCR",Float64,callbackOCR)
rospy.spin()		# main()
		







	