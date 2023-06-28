#!/usr/bin/env python2
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
	if msg.data>95 :
			msd()








def msd():


	fig,ax=plt.subplots(figsize=(12, 10))


	ratio=np.array([[round(FCR,3),round(OCR,3)]])
	print(ratio)
	
	ax.plot(coverage_percentagetime,coverage_percentage)
	ax.set_title("Coverage")
	ax.set_xlabel("Time(s)")
	ax.set_ylabel("Coverage Percentage %")
	table=ax.table(cellText=ratio,colLabels=('FCR','OCR'),loc='bottom',bbox=[0.0,-0.18,0.5,0.1])
	table.auto_set_font_size(False)
	table.set_fontsize(12)

	# Adjust table cell widths
	table.scale(1, 2)
	plt.subplots_adjust(bottom=0.2)
	fig.show()



	# np.append(particle1_msd,robot_0_pose)
	rospy.spin()





global coverage_percentage
global coverage_percentagetime
coverage_percentagetime=[]
coverage_percentage=[]


rospy.init_node('msd_calc', anonymous=True)



# Set the retry interval in seconds
retry_interval = 1

# Check if a publisher is available for the topic
while not rospy.is_shutdown():
    try:
        rospy.wait_for_message('/coverage_percentage', Float64, timeout=1)
        rospy.wait_for_message('/FCR', Float64, timeout=1)
        rospy.wait_for_message('/OCR', Float64, timeout=1)

        break
    except rospy.ROSException:
        rospy.logwarn("No publisher available for topic /coverage_percentage. Retrying in 1 seconds...")
        time.sleep(retry_interval)



sub = rospy.Subscriber("/coverage_percentage",Float64,callback)
subFCR = rospy.Subscriber("/FCR",Float64,callbackFCR)
subOCR = rospy.Subscriber("/OCR",Float64,callbackOCR)



rospy.spin()		







	