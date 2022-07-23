#!/usr/bin/env python
import  rospy
import  sys
import  numpy as np
from    std_msgs.msg import String,Float64
from    geometry_msgs.msg import Twist,Point,Pose
from    nav_msgs.msg import Odometry
from    robot_class import robot
from    laser_class import laser
from    math import pi
import  time 
from    datetime import datetime 
import  os

def kill_node(event):
    print 'Timer called at ' + str(event.current_real)
    os.system("rosnode kill "+ "rw_node")

def kill_node_manual():
    os.system("rosnode kill "+ "rw_node")
    
def coverage_callback(msg):
    global coverage_percentage    
    coverage_percentage=msg.data

def main():
    VonMisesKappa=rospy.get_param("/swarm/VonMisesKappa")
    VonMisesMu=rospy.get_param("/swarm/VonMisesMu")
    flag=0
    flag1=0
    # rospy.Timer(rospy.Duration(600), kill_node)
    t0= datetime.now()
    
    while not rospy.is_shutdown()  :
      
        step=6
        odom=r.get_odom()
        distance=0
        new_heading=np.random.vonmises(VonMisesMu,VonMisesKappa)
        r.fix_yaw(new_heading)    
        while step-distance>0.05 and not rospy.is_shutdown():
            rate.sleep()

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
            # rospy.loginfo(l.get_front_min_range())
            if l.get_front_min_range()<1:
                new_heading=np.random.vonmises(VonMisesMu,VonMisesKappa)  
                r.fix_yaw(new_heading)
                odom=r.get_odom()
                distance=r.euclidean_distance(odom)
            else:
                r.publish_vel(0.2,0)
                distance=r.euclidean_distance(odom)
    # kill_node_manual()


if __name__ == '__main__':
    try:
        while not  any('/coverage_percentage' in subl for subl in rospy.get_published_topics()) :
            print("The coverage_percentage topic is not published. Retrying.........")
            rospy.sleep(1)
        args=rospy.myargv(argv=sys.argv)
        robotname= args[1]
        rospy.init_node('Random_Walk', anonymous=True)
        # print(rospy.get_published_topics())
        subs = rospy.Subscriber("/coverage_percentage",Float64,coverage_callback)
   
        l=laser(robotname)
        r=robot(robotname)

        rate = rospy.Rate(5)
        rate.sleep()
        main()
    except rospy.ROSInterruptException:
        pass