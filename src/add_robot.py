#!/usr/bin/env python2
import  rospy
import  os
import roslaunch
import  time 

#f = open(os.path.dirname(__file__) + '/../world/test.world', "w")
f = open(os.path.split(os.path.dirname(__file__))[0] + '/world/test.world', "w")
print(os.path.split(os.path.dirname(__file__))[0] + '/world/test.world')
robots=rospy.get_param("/swarm/robots")
formation=rospy.get_param("/swarm/formation")
map=rospy.get_param("/swarm/map")
print(robots)
if (robots == None):
     ROS_ERROR("Error in getting the robots param\n");

f.write("""
  define block model
(
  size [20 0.5 0.5]
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
  size [10 0.5 0.5]
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
   samples 360
  )

  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)
define topurg1 ranger
(
  sensor(       
    range [ 0.0  4.5 ]
    fov 360
    ranger_return 1.0
   samples 360
  )

  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define erratic position
(
  #size [0.415 0.392 0.25]
  size [0.25 0.25 0.25]
  ranger_return 1.0
  origin [-0.05 0 0 0]
  gui_nose 1
  drive "diff"
  odom_error [0.0 0.0 0.00 0.0]
  topurg(pose [ 0.050 0.000 -0.2 0.000 ])
  topurg1(pose [ 0.050 0.000 0 0.000 ])
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
  size [ 1000.000 1000.000 ] 

  rotate [ 0.000 0.0 ]
  scale 20 
  show_grid 0
)
""")
if (map=="box") :
  f.write("""
  block( pose [ 0 10 0 0 ] color "black")
  block( pose [ 10 0 0 90 ] color "black")
  block( pose [ 0 -10 0 0 ] color "black")
  block( pose [ -10 0 0 90 ] color "black")
  block1( pose [ 3.5 2.5 0 0 ] color "black")
  block1( pose [ -3.5 2.5 0 0 ] color "black")
  block1( pose [ 3.5 -2.5 0 0 ] color "black")
  block1( pose [ -3.5 -2.5 0 0 ] color "black")
  block1( pose [ 5.5 6.5 0 0 ] color "black")
  block1( pose [ -5.5 6.5 0 0 ] color "black")
  block1( pose [ 5.5 -6.5 0 0 ] color "black")
  block1( pose [ -5.5 -6.5 0 0 ] color "black")



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
   


