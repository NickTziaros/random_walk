#!/usr/bin/env python2
import  rospy
import  os
import roslaunch
import  time 


f = open(os.path.split(os.path.dirname(__file__))[0] + '/launch/test_slam.launch', "w")
robots=rospy.get_param("/swarm/robots")
map=rospy.get_param("/swarm/map")


f.write('<launch>\n<arg name="ns" default="robot_1"/>\n')

for i in range(robots):
	f.write('<remap from="/robot_' + str(i)+'/scan" to ="/robot_' + str(i)+'/base_scan_1" />\n')


	if (map=="corridor"):
		xmax=12
		xmin=-12
		ymax=20
		ymin=-20
	elif (map=="box"):
		xmax=12
		xmin=-12
		ymax=12
		ymin=-12
	elif (map=="empty"):
		xmax=50
		xmin=-50
		ymax=50
		ymin=-50
for i in range(robots):
	f.write('''
	<node pkg="gmapping" type="slam_gmapping" name="stage_slam_gmapping" output="screen" ns="robot_'''+ str(i)+'''">
    	<param name="base_frame" value="/robot_''' + str(i)+'/base_footprint"/>\n'+
    '		<param name="odom_frame" value="/robot_' + str(i)+'/odom"/>\n'+
    '		<param name="map_frame"  value="/robot_' + str(i)+'/map"/>\n'+
    '''		<param name="map_update_interval" value="2.0"/>
    	<param name="maxUrange" value="4"/>
    	<param name="minimumScore" value="150"/>
    	<param name="linearUpdate" value="0.1"/>
    	<param name="angularUpdate" value="0.1"/>
    	<param name="temporalUpdate" value="-0.5"/>
    	<param name="delta" value="0.05"/>
    	<param name="lskip" value="0"/>
    	<param name="particles" value="30"/>
    	<param name="sigma" value="0.05"/>
    	<param name="kernelSize" value="1"/>
    	<param name="lstep" value="0.05"/>
    	<param name="astep" value="0.05"/>
    	<param name="iterations" value="10"/>
    	<param name="lsigma" value="0.075"/>
    	<param name="ogain" value="5.0"/>
    	<param name="srr" value="0"/>
    	<param name="srt" value="0"/>
    	<param name="str" value="0"/>
    	<param name="stt" value="0"/>
    	<param name="resampleThreshold" value="0.5"/>
    	<param name="xmin" value="'''+ str(xmin)+'''"/>
    	<param name="ymin" value="'''+ str(ymin)+'''"/>
    	<param name="xmax" value="'''+ str(xmax)+'''"/>
    	<param name="ymax" value="'''+ str(ymax)+'''"/>
    	<param name="llsamplerange" value="0.01"/>
    	<param name="llsamplestep" value="0.01"/>
    	<param name="lasamplerange" value="0.005"/>
    	<param name="lasamplestep" value="0.005"/>
    	<param name="maxRange" value="80"/>
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
f.close()

f = open(os.path.split(os.path.dirname(__file__))[0] + '/launch/odom_to_path.launch', "w")
f.write("<launch>\n")
f.write('<arg name="robot_name" default="robot"/>\n')

for i in range(robots):
	
	f.write('''<group ns="$(arg robot_name)_'''+str(i)+'''">\n
	<node pkg="random_walk" name="odom_to_path" type="odom_to_path.py" output="screen" args="$(arg robot_name)_'''+str(i)+''' "></node> \n

	</group>\n''')

f.write("</launch>")
f.close()