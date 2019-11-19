#!/usr/bin/env python

import image1
import image2
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class Server:
    def __init__(self):
        self.angles1 = None
        self.angles2 = None

    def image1_callback(self, msg):
        self.angles1 = msg
        self.compute_average()

    def image2_callback(self, msg):
        self.angles2 = msg
        self.compute_average()

    def compute_average(self):
        if self.angles1 is not None and self.angles2 is not None:
            print("Angles1: ", self.angles1.data)
            print("Angles2: ", self.angles2.data)
	    print("Average: ", np.array([np.mean([i, j]) for (i, j) in zip(self.angles1.data, self.angles2.data)]))
	    print()	
	

if __name__ == "__main__":
    server = Server()

    rospy.init_node('angles')
    rospy.Subscriber("/joints_pos1", Float64MultiArray, server.image1_callback)
    rospy.Subscriber("/joints_pos2", Float64MultiArray, server.image2_callback)

    rospy.spin()
