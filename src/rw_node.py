#!/usr/bin/env python
import	rospy
import 	sys
import  numpy as np
from 	geometry_msgs.msg import Twist,Point,Pose
from 	nav_msgs.msg import Odometry
from	robot_class import robot
from    laser_class import laser
from    math import pi
import  time    


def main():

    while not rospy.is_shutdown():
        step=0.5
        odom=r.get_odom()
        distance=0
        new_heading=np.random.vonmises(0,0.4)
        r.fix_yaw(new_heading)
        r.publish_vel(0.0,0)
        while l.closest_point()<0.5 and not rospy.is_shutdown():
            r.publish_vel(0,0)
            obst_yaw=r.obst_yaw(l.closest_point())
            new_heading=np.random.vonmises(0,0.4)
            fix_yaw(r.get_yaw()-obst_yaw)
            r.publish_vel(0.5,0)
            sleep(3)
        
        while step-distance>0.05 and not rospy.is_shutdown():
            r.publish_vel(0.2,0)
            distance=r.euclidean_distance(odom)
            rospy.loginfo(distance)
            if l.closest_point()<0.5:
                break




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







    


