#!/usr/bin/env python
import  rospy
from  math import cos,sin
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point,PointStamped,PoseStamped
import tf
import tf2_ros
import tf2_geometry_msgs



class laser():
	
	def __init__(self,robotname):


		self.robotname=robotname

		#Gazebo 
		# self.subs = rospy.Subscriber("/{}/scan".format(robotname),LaserScan,self.Laser_callback)

		# Stage
		self.subs = rospy.Subscriber("/{}/base_scan_0".format(robotname),LaserScan,self.Laser_callback)

		self.closest_point_pub = rospy.Publisher("/{}/closest_point".format(robotname),PointStamped, queue_size=20)
		# rate=rospy.Rate(50)
		# rate.sleep()
		# subscribres to TF and listens at the transforms tha are published
		self.listener=tf.TransformListener()
		# we wait for the tranformations between sensor_laser and odom

		# Gazebo
		# self.listener.waitForTransform("/{}/base_scan".format(self.robotname), "/{}/odom".format(self.robotname), rospy.Time(0),rospy.Duration(5))

		# Stage
		self.listener.waitForTransform("/{}/base_laser_link_0".format(self.robotname), "/{}/odom".format(self.robotname), rospy.Time(0),rospy.Duration(15))




	



	def Laser_callback(self,msg):
		self.laser=msg
		
	def get_front_min_range(self):
		ranges_list=self.laser.ranges
		#Gazebo
		# front_rays=ranges_list[0:34]+ranges_list[325:359]
		#Stage
		front_rays=ranges_list[145:215]
		min_range=min(front_rays)
		return min_range
	def get_front_max_range(self):
		ranges_list=self.laser.ranges
		#Gazebo
		# front_rays=ranges_list[0:60]+ranges_list[300:359]
		#Stage
		front_rays=ranges_list[145:215]
		max_range=max(front_rays)
		return max_range
	def closest_point(self):
		
		
		# we get the closest reading from the laser scan
		laser=self.laser.ranges
		shortest_laser=100000
		laser_point=Point()
		for i in range(len(laser)):
			if laser[i]<shortest_laser:
				shortest_laser=laser[i]
				angle=self.laser.angle_min + i*self.laser.angle_increment
				x=laser[i]*cos(angle)
				laser_point.x=x
				laser_point.y=shortest_laser*sin(angle)
		pose=PoseStamped()
		pose.header=self.laser.header
		pose.pose.position = laser_point


		

		point_transformed=PointStamped()
		# Gazebo
		# point_transformed.header.frame_id="/{}/base_scan".format(self.robotname)
		# Stage
		point_transformed.header.frame_id="/{}/base_laser_link_0".format(self.robotname)
		point_transformed.header.stamp= rospy.Time(0) 
		# we include the proint we found
		point_transformed.point=laser_point

		try:
			# we transform the point to odom frame
			p=self.listener.transformPoint("/{}/odom".format(self.robotname),point_transformed )
		except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
			print ("ERROR")


		# we publish the transformed point
		self.closest_point_pub.publish(p)
		laser_point.x=p.point.x
		laser_point.y=p.point.y
		


		# returns the point closer to the robot type=Point()
		return	laser_point