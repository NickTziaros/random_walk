#!/usr/bin/env python
import	rospy
import 	sys
import  numpy as np
from 	geometry_msgs.msg import Twist,Point,Pose
from 	nav_msgs.msg import Odometry
from	robot_class import robot
from    laser_class import laser
import  time    


def main():
    

    while not rospy.is_shutdown():
        step=0.5
        # if l.closest_point() < 0.6:
        odom=r.get_odom()
        distance=0
        new_heading=np.random.vonmises(0,0.4)
        r.fix_yaw(new_heading)
        r.publish_vel(0.0,0)
        time.sleep(6)
        rospy.loginfo("lol")
        r.publish_vel(0,0)
        rospy.loginfo(distance)
        if step-distance>0.05 :
            r.publish_vel(0.1,0)
            distance=r.euclidean_distance(odom)
            # rospy.loginfo(distance)

        # else:
        #     new_heading=np.random.vonmises(0,0.4)
        #     rospy.loginfo(new_heading)

        #     r.publish_vel(0.0,0)
        #     r.fix_yaw(new_heading)
        



        # print(r.get_odom())
        # print(l.closest_point())


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







    


