import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nikos/catkin_ws/src/random_walk/launch/stage.launch"])
launch.start()
rospy.sleep(2)
launch1 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nikos/catkin_ws/src/random_walk/launch/test_slam.launch"])
launch1.start()
rospy.sleep(2)
launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nikos/catkin_ws/src/random_walk/launch/test_mapmerge.launch"])
launch2.start()
# rospy.sleep(3)
# launch3 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nikos/catkin_ws/src/random_walk/launch/testrw.launch"])
# launch3.start()
# rospy.loginfo("started")

# rospy.sleep(3)
while not rospy.is_shutdown()  :
	rospy.sleep(1)