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

from .inverse_kinematics import *


class Server:
    def __init__(self):
        self.zs1 = None
        self.ys = None
        self.zs2 = None
        self.xs = None
        self.previous_angles = np.zeros(0)
        self.previous_effector_pos = np.array([0, 0, 7])

    def image1_callback_y(self, msg):
        self.ys = msg
        self.calculate_angles()

    def image1_callback_z(self, msg):
        self.zs1 = msg
        self.calculate_angles()

    def image2_callback_x(self, msg):
        self.xs = msg
        self.calculate_angles()

    def image2_callback_z(self, msg):
        self.zs2 = msg
        self.calculate_angles()

    def calculate_angles(self):
        if self.zs1 is not None and self.ys is not None and self.zs2 is not None and self.xs is not None:
            # print("Image1: ", zip(self.ys.data, self.zs1.data))
            # print("Image2: ", zip(self.xs.data, self.zs2.data))
            mean_zs = [np.mean([z1, z2]) for (z1, z2) in zip(self.zs1.data, self.zs2.data)]
            xs = self.xs.data
            ys = self.ys.data

            # joint positions
            effector_pos = np.array([xs[3], ys[3], mean_zs[3]])
            print("3: ", effector_pos)

            predicted_angles = solve_angles(effector_pos)

            if self.check_z_coordinate(effector_pos[2]):
                predicted_angles[0] += find_angle_joint_1(self.previous_effector_pos[:2], effector_pos[:2])

            print(predicted_angles)

            self.previous_effector_pos = effector_pos
            self.previous_angles = predicted_angles

    def check_z_coordinate(self, predicted_z):
        diff = np.abs(self.previous_effector_pos[2] - predicted_z)
        return diff <= 0.8


if __name__ == "__main__":
    server = Server()

    rospy.init_node('angles')
    rospy.Subscriber("/joints_pos_y", Float64MultiArray, server.image1_callback_y)
    rospy.Subscriber("/joints_pos_z_1", Float64MultiArray, server.image1_callback_z)
    rospy.Subscriber("/joints_pos_x", Float64MultiArray, server.image2_callback_x)
    rospy.Subscriber("/joints_pos_z_2", Float64MultiArray, server.image2_callback_z)

    rospy.spin()
