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
import  os

def kill_node(event):
    print 'Timer called at ' + str(event.current_real)
    os.system("rosnode kill "+ "rw_node")

def kill_node_manual():
    os.system("rosnode kill "+ "rw_node")
    


def main():
    rospy.Timer(rospy.Duration(600), kill_node)
    coverage_percentage=r.get_coverage_percentage()
    while not rospy.is_shutdown() and coverage_percentage<80:
      
        step=5
        odom=r.get_odom()
        distance=0
        new_heading=np.random.vonmises(0,0)
        r.fix_yaw(new_heading)    
        while step-distance>0.05 and not rospy.is_shutdown():
            rate.sleep()
            coverage_percentage=r.get_coverage_percentage()
            # rospy.loginfo(l.get_front_min_range())
            if l.get_front_min_range()<2:
                new_heading=np.random.vonmises(0,0)  
                r.fix_yaw(new_heading)
                odom=r.get_odom()
                distance=r.euclidean_distance(odom)
            else:
                r.publish_vel(0.2,0)
                distance=r.euclidean_distance(odom)
    kill_node_manual()


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