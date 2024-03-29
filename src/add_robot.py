#!/usr/bin/env python2
import  rospy
import  os
import roslaunch
import  time 

#f = open(os.path.dirname(__file__) + '/../world/test.world', "w")
f = open(os.path.split(os.path.dirname(__file__))[0] + '/world/test1.world', "w")
print(os.path.split(os.path.dirname(__file__))[0] + '/world/test1.world')
robots=rospy.get_param("/swarm/robots")
formation=rospy.get_param("/swarm/formation")
map=rospy.get_param("/swarm/map")
print(robots)
if (robots == None):
     ROS_ERROR("Error in getting the robots param\n");

f.write("""
 define block model
(
  size [21.000 0.500 0.500]
  gui_nose 0
)
  define block3 model
(
  size [40 0.5 0.5]
  gui_nose 0
)
define block1 model
(
  size [1 1 1]
  gui_nose 0
)

define block2 model
(
  size [4.000 1.000 1.500]
  gui_nose 0
)
define block5 model
(
  size [40 0.5 0.5]
  gui_nose 0
)
define topurg ranger
(
  sensor(       
    range [ 0.0  4.5 ]
    fov 360
    ranger_return 1.0
   samples 720
  )

  # generic model properties
  color "black"
  size [ 0.050 0.050 0.100 ]
)
define topurg1 ranger
(
  sensor(       
    range [ 0.0  4.5 ]
    fov 360
    ranger_return 1.0
   samples 720
  )

  # generic model properties
  color "black"
  size [ 0.050 0.050 0.100 ]
)

define erratic position
(
  #size [0.415 0.392 0.25]
  size [0.250 0.250 0.250]
  ranger_return 1.000
  origin [-0.050 0.000 0.000 0.000]
  gui_nose 1
  drive "diff"
  odom_error [0.0 0.0 0.00 0.0]
  topurg(pose [ 0.050 0.000 -0.200 0.000 ])
  topurg1(pose [ 0.050 0.000 0.000 0.000 ])
)

define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0

  gui_outline 0
  gripper_return 0
  fiducial_return 0
  ranger_return 1
)

# set the resolution of the underlying raytrace model in meters
resolution 0.02

interval_sim 100  # simulation timestep in milliseconds


window
( 
  size [ 1000 1000 ] 

  rotate [ 0.000 0.000 ]
  scale 38.974 
  show_grid 1
)
""")
if (map=="box") :
  f.write("""
  block( pose [ 0.062 10.279 0.000 0.000 ] color "black")
  block( pose [ 10.350 -0.018 0.000 90.000 ] color "black")
  block( pose [ 0.062 -10.273 0.000 0.000 ] color "black")
  block( pose [ -10.217 0.000 0.000 90.000 ] color "black")
  block2( pose [ 4.081 2.500 0.000 0.000 ] color "black")
  block2( pose [ -1.557 -5.008 0.000 90.000 ] color "black")
  block2( pose [ 1.564 5.008 0.000 90.000 ] color "black")
  block2( pose [ -4.057 -2.482 0.000 0.000 ] color "black")
  #block1( pose [ 5.5 6.5 0 0 ] color "black")
  #block1( pose [ -5.5 6.5 0 0 ] color "black")
  #block1( pose [ 5.5 -6.5 0 0 ] color "black")
  #block1( pose [ -5.5 -6.5 0 0 ] color "black")



  """)
elif (map=="corridor") :
  f.write("""
  block2( pose [ 0 20 0 0 ] color "black")
  block3( pose [ 5 0 0 90 ] color "black")
  block2( pose [ 0 -20 0 0 ] color "black")
  block3( pose [ -5 0 0 90 ] color "black")
  



  """)



elif (map=="empty") :
  f.write("""
  block5( pose [ 0 20 0 0 ] color "black")
  block5( pose [ 20 0 0 90 ] color "black")
  block5( pose [ 0 -20 0 0 ] color "black")
  block5( pose [ -20 0 0 90 ] color "black")
  



  """)






if (formation=="no") :
  for i in range(robots):
    f.write("erratic( pose [  "+str(i)+' 0  0 0  0] name "era'+str(i)+'" color "blue")\n')

elif (formation=="box"):
  j=0
  k=-2
  for i in range(robots):
    if k==2 :
      k=-2
      j=j-0.5
    f.write("erratic( pose [  "+str(k)+" "+str(j)+'  0 0  0] name "era'+str(i)+'" color "blue")\n')
    k=k+1

# elif (formation=="pyramid"):
#   r = 0
#   c=1
#   # count=0
#   # count1=0
#   # rows=0
#   for i in range(robots):
    
#     f.write("erratic( pose [  "+str(r)+" "+str(c)+'  0 0  0] name "era'+str(i)+'" color "blue")\n')



#     r=r+0.5
#     c=c+1

#   print(rows)
# Python 3.x code to demonstrate star pattern






# Function to demonstrate printing pattern triangle



  # f.write("erratic( pose [  "+str(k)+" "+str(j)+'  0 0  0] name "era'+str(i)+'" color "blue")\n')
   


