#!/usr/bin/env python2

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from collections import deque
import  numpy as np
    
class CoveragePlotter:

    def __init__(self):
        # Initialize the node
        rospy.init_node('coverage_plotter')
        # Check if a publisher is available for the topic
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message('/coverage_percentage', Float64, timeout=1)
                rospy.wait_for_message('/FCR', Float64, timeout=1)
                rospy.wait_for_message('/OCR', Float64, timeout=1)
                rospy.loginfo('Got messages for topic /coverage_percentage,/FCR,/OCR')

                break
            except rospy.ROSException:
                rospy.logwarn("No publisher available for topic /coverage_percentage. Retrying in 1 seconds...")
        # Initialize the data storage deque
        self.OCR = deque(maxlen=100)
        self.FCR = deque(maxlen=100)

        self.data = deque(maxlen=100)

        self.rate = rospy.Rate(1)
        # Initialize the plot
        plt.ion()
        self.fig, self.ax = plt.subplots(3,1,figsize=(6, 6))
        # self.ax.set_xlabel('Time')
        # self.ax.set_ylabel('Coverage Percentage')
        # self.ax.set_ylim([0, 100])


    
        # Subscribe to the /coverage_percentage topic
        self.subscriber = rospy.Subscriber('/coverage_percentage', Float64, self.callback)
        self.subFCR = rospy.Subscriber("/FCR",Float64,self.callbackFCR)
        self.subOCR = rospy.Subscriber("/OCR",Float64,self.callbackOCR)
        self.rate.sleep()

    def callbackFCR(self,msg):
        # self.FCR=msg.data
        self.FCR.append((rospy.Time.now().to_sec(), msg.data))
        # print(FCR)

    def callbackOCR(self,msg):
        # self.OCR=msg.data
        self.OCR.append((rospy.Time.now().to_sec(), msg.data))
        # print(OCR)

    def callback(self, msg):
        # Store the received data
        self.data.append((rospy.Time.now().to_sec(), msg.data))

    def update_plot(self):
        # Clear the axis and plot the data
        self.ax[0].clear()
        self.ax[0].set_xlabel('Time')
        self.ax[0].set_ylabel('Coverage Percentage %')
        self.ax[0].set_ylim([0, 100])
        x, y = zip(*self.data)
        self.ax[0].plot(x, y)
        # self.table=self.ax.table(cellText=ratio,colLabels=('FCR','OCR'),bbox=[0.0,-0.13,0.2,0.1])
        # self.table.auto_set_font_size(False)
        self.ax[1].clear()
        self.ax[1].set_xlabel('Time')
        self.ax[1].set_ylabel('OCR %')
        self.ax[1].set_ylim([0, 1])
        x, y = zip(*self.OCR)
        self.ax[1].plot(x, y)

        self.ax[2].clear()
        self.ax[2].set_xlabel('Time')
        self.ax[2].set_ylabel('FCR %')
        self.ax[2].set_ylim([0, 1])
        x, y = zip(*self.FCR)
        self.ax[2].plot(x, y)
        # Draw the updated plot
        self.fig.canvas.draw()

    def run(self):
        # Continuously update the plot
        while not rospy.is_shutdown():
            self.rate.sleep()

            if len(self.data) > 0:
                self.update_plot()
            plt.pause(0.1)

if __name__ == '__main__':
    try:
        plotter = CoveragePlotter()
        rate = rospy.Rate(1)
        rate.sleep()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
