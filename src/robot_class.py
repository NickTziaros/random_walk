#!/usr/bin/env python
import  rospy
from    geometry_msgs.msg import Twist,Point,Pose
from    nav_msgs.msg import Odometry
from    math    import sqrt,pow,atan2,pi,cos,sin
from    tf.transformations import euler_from_quaternion
from    random import *
import  numpy

class robot():

	def __init__(self,robotname):
		self.robotname=robotname
		self.subs = rospy.Subscriber("/{}/odom".format(robotname),Odometry,self.sub_callback)
		self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
		self.vel=Twist()
		
	def sub_callback(self,msg):
		self.robot_pose=[msg.pose.pose.position.x,msg.pose.pose.position.y]


	def publish_vel(self,linear,angular):
		self.vel.linear.x=linear
		self.vel.angular.z=angular
		self.pub.publish(self.vel)
	
	def print_odom(self):
		rospy.loginfo(self.robot_pose[0])
		rospy.loginfo(self.robot_pose[1])
