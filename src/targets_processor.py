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
        self.z1 = None
        self.z2 = None
	self.target_z = rospy.Publisher("target_z", Float64, queue_size=10)

    def image1_callback(self, msg):
        self.z1 = msg
        self.compute_average()

    def image2_callback(self, msg):
        self.z2 = msg
        self.compute_average()

    def compute_average(self):
        if self.z1 is not None and self.z2 is not None:
            #print("z1: ", self.z1.data)
            #print("z2: ", self.z2.data)
	    print("Average: ", np.mean([self.z1.data, self.z2.data]))
	    print()
	    mean_z = Float64()
	    mean_z.data = np.mean([self.z1.data, self.z2.data])
	    self.target_z.publish(mean_z)	
	

if __name__ == "__main__":
    server = Server()

    rospy.init_node('zs')
    rospy.Subscriber("/target_z_1", Float64, server.image1_callback)
    rospy.Subscriber("/target_z_2", Float64, server.image2_callback)

    rospy.spin()
