#!/usr/bin/env python
import	rospy
import 	sys
from 	geometry_msgs.msg import Twist,Point,Pose
from 	nav_msgs.msg import Odometry
from	robot_class import robot
from    laser_class import laser


def main():
    

    while not rospy.is_shutdown():
        r.publish_vel(0.1,0.2) 
        odom=r.get_odom()
        print(r.get_odom())
        print(l.closest_point())


if __name__ == '__main__':
    try:
        args=rospy.myargv(argv=sys.argv)
        robotname= args[1]
        rospy.init_node('Random_Walk', anonymous=True)
        r=robot(robotname)
        l=laser(robotname)
        main()
    except rospy.ROSInterruptException:
        pass







    


