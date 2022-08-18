#!/usr/bin/env python
import  rospy
from    geometry_msgs.msg import Twist,Point,Pose
from    nav_msgs.msg import Odometry
from 	std_msgs.msg import String,Float64
from    math    import sqrt,pow,atan2,pi,cos,sin
from 	tf import transformations
from    tf.transformations import euler_from_quaternion
from    random import *
import  numpy
import  math

class robot():

	def __init__(self,robotname):
		self.robotname=robotname
		self.subs = rospy.Subscriber("/{}/odom".format(robotname),Odometry,self.sub_callback)
		# self.subs = rospy.Subscriber("/coverage_percentage",Float64,self.coverage_callback)
		self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
		self.vel=Twist()
		self.rate = rospy.Rate(10)
		self.rate.sleep()
        
	# def coverage_callback(self,msg):	
	# 	self.coverage_percentage=msg.data
# -----------------------------------------------------------------------------------------
		
	def sub_callback(self,msg):
		self.robot_pose=[msg.pose.pose.position.x,msg.pose.pose.position.y]
		rot_q= msg.pose.pose.orientation
		(roll,pitch,self.yaw)= euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w]) 
		

# -----------------------------------------------------------------------------------------

	def publish_vel(self,linear,angular):
		self.vel.linear.x=linear
		self.vel.angular.z=angular
		self.pub.publish(self.vel)

# -----------------------------------------------------------------------------------------

	def get_odom(self):
		# rospy.loginfo(self.robot_pose[0])
		# rospy.loginfo(self.robot_pose[1])
		return self.robot_pose

# -----------------------------------------------------------------------------------------

	def obst_yaw(self,obst):
		yaw=atan2(obst.y- self.robot_pose[1],obst.x- self.robot_pose[0])
		return yaw

# -----------------------------------------------------------------------------------------

	def get_yaw_deg(self):
		return self.rad2deg(self.yaw)

# -----------------------------------------------------------------------------------------

	def get_yaw(self):
		return self.yaw

# -----------------------------------------------------------------------------------------

	# def get_coverage_percentage(self):
	# 	return self.coverage_percentage




# -----------------------------------------------------------------------------------------

	def euclidean_distance(self, goal_point):
		distance= sqrt(pow((goal_point[0] - self.robot_pose[0]), 2) +
				pow((goal_point[1] - self.robot_pose[1]), 2))

		return distance
# -----------------------------------------------------------------------------------------		
	def euclidean_distance_laser(self, goal_point):
		distance= sqrt(pow((goal_point.x - self.robot_pose[0]), 2) +
				pow((goal_point.y - self.robot_pose[1]), 2))

		return distance
# -----------------------------------------------------------------------------------------

	def rad2deg(self,rad):
		deg=round((rad*180)/pi,4)
		return deg

# -----------------------------------------------------------------------------------------

	def fix_yaw(self,des_yaw):
		self.publish_vel(0.0,0)
		# rospy.loginfo(self.rad2deg(des_yaw))
		yaw_deg=self.get_yaw_deg()
		angle_diff= self.rad2deg(des_yaw) - yaw_deg
		while abs(angle_diff )>5  and not rospy.is_shutdown():
			self.rate.sleep()

			if angle_diff < 0: 
				speed=-0.4
			else:
				speed= 0.4
			angle_diff= self.rad2deg(des_yaw) - self.get_yaw_deg()
			# rospy.loginfo(angle_diff)
			self.publish_vel(0,speed)

