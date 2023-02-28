#!/usr/bin/env python2
import  rospy
import  os
import roslaunch
import  time 

rospy.init_node('Add Maps', anonymous=True)
f = open(os.path.split(os.path.dirname(__file__))[0] + '/launch/test_mapmerge.launch', "w")
robots=rospy.get_param("/swarm/robots")
formation=rospy.get_param("/swarm/formation")
print(robots)

if (robots < 2):
     rospy.logerr("multirobot_map_merge needs more than two robots to \n");
if (formation != "box" and formation !="no"):
     rospy.logerr("Invalid formation parameter value\n");

f.write("<launch>\n")
for i in range(robots):
	
	f.write('<arg name="'+ str(i)+'_robot"  default="robot_'+ str(i)+'"/>\n')


if (formation=="no") :
  for i in range(robots):
     f.write('''
     <group ns="/robot_'''+ str(i)+'''/map_merge">
     <param name="init_pose_x" value="'''+ str(i)+'''"/>
     <param name="init_pose_y" value="0"/>
     <param name="init_pose_z" value=" 0.0"/>
     <param name="init_pose_yaw"   value="0"/>
     </group>''') 

elif (formation=="box"):
     j=0
     k=-2
     for i in range(robots):
          if k==2:
               k=-2
               j=j-0.5

          f.write('''
          <group ns="/robot_'''+ str(i)+'''/map_merge">
          <param name="init_pose_x" value="'''+ str(k)+'''"/>
          <param name="init_pose_y" value="'''+ str(j)+'''"/>
          <param name="init_pose_z" value=" 0.0"/>
          <param name="init_pose_yaw"   value="0"/>
          </group>''')

          k=k+1



f.write('''
	<node pkg="multirobot_map_merge" type="map_merge" respawn="false" name="map_merge" output="screen">
      <param name="robot_map_topic" value="map"/>
      <param name="robot_namespace" value=""/>
      <param name="merged_map_topic" value="merged_map"/>
      <param name="world_frame" value="/map"/>
      <param name="known_init_poses" value="true"/>
      <param name="merging_rate" value="15"/>
      <param name="discovery_rate" value="0.05"/>
      <param name="estimation_rate" value="0.5"/>
      <param name="estimation_confidence" value="1.0"/>
    </node>\n''')


for i in range(robots):
	f.write('  <node pkg="tf" type="static_transform_publisher" name="world_to_$(arg '+ str(i)+'_robot)_tf_broadcaster"  args="0 0 0 0 0 0 /map /$(arg '+ str(i)+'_robot)/map 200"/>\n')


f.write("</launch>")
