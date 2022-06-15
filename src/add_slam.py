#!/usr/bin/env python
import  rospy
import  os
import roslaunch
import  time 


f = open(os.path.split(os.path.dirname(__file__))[0] + '/launch/test_slam.launch', "w")
robots=rospy.get_param("/swarm/robots")
print(robots)

if (robots == None):
     ROS_ERROR("Error in getting the robots param\n");

f.write('<launch>\n<arg name="ns" default="robot_1"/>\n')

for i in range(robots):
	f.write('<remap from="/robot_' + str(i)+'/scan" to ="/robot_' + str(i)+'/base_scan_1" />\n')

for i in range(robots):
	f.write('''
	<node pkg="gmapping" type="slam_gmapping" name="stage_slam_gmapping" output="screen" ns="robot_'''+ str(i)+'''">
    	<param name="base_frame" value="/robot_''' + str(i)+'/base_footprint"/>\n'+
    '		<param name="odom_frame" value="/robot_' + str(i)+'/odom"/>\n'+
    '		<param name="map_frame"  value="/robot_' + str(i)+'/map"/>\n'+
    '''		<param name="map_update_interval" value="2.0"/>
    	<param name="maxUrange" value="59.99"/>
    	<param name="minimumScore" value="100"/>
    	<param name="linearUpdate" value="1"/>
    	<param name="angularUpdate" value="1"/>
    	<param name="temporalUpdate" value="-0.5"/>
    	<param name="delta" value="0.05"/>
    	<param name="lskip" value="0"/>
    	<param name="particles" value="30"/>
    	<param name="sigma" value="0.05"/>
    	<param name="kernelSize" value="1"/>
    	<param name="lstep" value="0.05"/>
    	<param name="astep" value="0.05"/>
    	<param name="iterations" value="5"/>
    	<param name="lsigma" value="0.075"/>
    	<param name="ogain" value="5.0"/>
    	<param name="srr" value="0.0"/>
    	<param name="srt" value="0.0"/>
    	<param name="str" value="0.0"/>
    	<param name="stt" value="0.0"/>
    	<param name="resampleThreshold" value="0.5"/>
    	<param name="xmin" value="-12.0"/>
    	<param name="ymin" value="-12.0"/>
    	<param name="xmax" value="12.0"/>
    	<param name="ymax" value="12.0"/>
    	<param name="llsamplerange" value="0.01"/>
    	<param name="llsamplestep" value="0.01"/>
    	<param name="lasamplerange" value="0.005"/>
    	<param name="lasamplestep" value="0.005"/>
    	<param name="maxRange" value="4.4"/>
	</node>\n''')
f.write("</launch>")

f.close()
f = open(os.path.split(os.path.dirname(__file__))[0] + '/launch/testrw.launch', "w")
f.write("<launch>\n")
f.write('<arg name="robot_name" default="robot"/>')

for i in range(robots):
	
	f.write('''<group ns="$(arg robot_name)_'''+str(i)+'''">\n
	<node pkg="random_walk" name="rw_node" type="rw_node.py" output="screen" args="$(arg robot_name)_'''+str(i)+''' "></node> \n

	</group>\n''')
f.write('<node pkg="random_walk" name="coverage_percentage" type="map_compare.py" output="screen"></node>\n')

f.write("</launch>")