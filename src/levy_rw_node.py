#!/usr/bin/env python
import  rospy
import  sys
import  numpy as np
from    geometry_msgs.msg import Twist,Point,Pose
from    nav_msgs.msg import Odometry
from    robot_class import robot
from    laser_class import laser
from    math import pi
from    scipy.stats import levy
from    datetime import datetime 
import  os
import  time    

def kill_node(event):

    os.system("rosnode kill "+ "levy_rw_node")

def kill_node_manual():
    os.system("rosnode kill "+ "levy_rw_node")
    

def main():
    VonMisesKappa=rospy.get_param("/swarm/VonMisesKappa")
    VonMisesMu=rospy.get_param("/swarm/VonMisesMu")
    flag=0
    flag1=0
    t0= datetime.now()
    # coverage_percentage=r.get_coverage_percentage()
    # rospy.Timer(rospy.Duration(60), kill_node)
    while not rospy.is_shutdown():
        step=levy.rvs(loc=6,scale=0.2)
        # rospy.loginfo("{}'s step= {}".format(robotname,step))
        odom=r.get_odom()
        distance=0
        new_heading=np.random.vonmises(VonMisesMu,VonMisesKappa)
        r.fix_yaw(new_heading)    
        while step-distance>0 and not rospy.is_shutdown():
            rate.sleep()
            coverage_percentage=r.get_coverage_percentage()
            if coverage_percentage>90:
                t1 = datetime.now() - t0
                print("time to reach " + str(90) + "%" + "coverage is: "+ str(t1))
                kill_node_manual()
            if coverage_percentage>50 and flag==0:
                t2 = datetime.now() - t0
                flag=1
                print("time to reach " + str(50) + "%" + "coverage is: "+ str(t2))
            if coverage_percentage>25 and flag1==0:
                t3 = datetime.now() - t0
                flag1=1
                print("time to reach " + str(25) + "%" + "coverage is: "+ str(t3))
            if l.get_front_min_range()<1:
                r.publish_vel(0,0)
                new_heading=np.random.vonmises(VonMisesMu,VonMisesKappa)
                r.fix_yaw(new_heading)
                step=levy.rvs(loc=6,scale=0.2)
                # rospy.loginfo("{}'s step= {}".format(robotname,step))
                odom=r.get_odom()
                distance=r.euclidean_distance(odom) 
            else:
                r.publish_vel(0.3,0)
                distance=r.euclidean_distance(odom)

if __name__ == '__main__':
    try:
        args=rospy.myargv(argv=sys.argv)
        robotname= args[1]
        rospy.init_node('Random_Walk', anonymous=True)
        l=laser(robotname)
        r=robot(robotname)
        rate = rospy.Rate(5)
        rate.sleep()
        main()
    except rospy.ROSInterruptException:
        pass