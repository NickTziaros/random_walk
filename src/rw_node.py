#!/usr/bin/env python
import  rospy
import  sys
import  numpy as np
from    geometry_msgs.msg import Twist,Point,Pose
from    nav_msgs.msg import Odometry
from    robot_class import robot
from    laser_class import laser
from    math import pi
import  time    


def main():

    while not rospy.is_shutdown():
        
        step=3
        odom=r.get_odom()
        distance=0
        new_heading=np.random.vonmises(0,0.4)
        r.fix_yaw(new_heading)    
        while step-distance>0.05 and not rospy.is_shutdown():
            r.publish_vel(0.4,0)
            distance=r.euclidean_distance(odom)
            # rospy.loginfo(l.get_front_min_range())
            if l.get_front_min_range()<0.4:
                new_heading=np.random.vonmises(0,0.4)  
                r.fix_yaw(new_heading)              
                # rospy.loginfo(r.rad2deg(obst_yaw))
                # rospy.loginfo("obst")
                # rospy.loginfo("yaw")
                # rospy.loginfo(r.get_yaw())

if __name__ == '__main__':
    try:
        args=rospy.myargv(argv=sys.argv)
        robotname= args[1]
        rospy.init_node('Random_Walk', anonymous=True)
        l=laser(robotname)
        r=robot(robotname)
        rate = rospy.Rate(10)
        rate.sleep()
        main()
    except rospy.ROSInterruptException:
        pass