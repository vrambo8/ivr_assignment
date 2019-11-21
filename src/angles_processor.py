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
        self.zs1 = None
        self.ys = None
        self.zs2 = None
        self.xs = None

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
            center = np.array([xs[0], ys[0], mean_zs[0]])
            print("Center: ", center)
            circle1Pos = np.array([xs[1], ys[1], mean_zs[1]])
            print("1: ", circle1Pos)
            circle2Pos = np.array([xs[2], ys[2], mean_zs[2]])
            print("2: ", circle2Pos)
            circle3Pos = np.array([xs[3], ys[3], mean_zs[3]])
            print("3: ", circle3Pos)

            # angles
            joint1 = center - circle1Pos
            ja1 = self.detect_joint_angle(np.array([0, 0, 1]), joint1)
            joint2 = circle1Pos - circle2Pos
            #ja2 = self.detect_joint_angle(joint1, joint2)
            joint3 = circle2Pos - circle3Pos
            ja3 = self.detect_joint_angle(joint2, joint3)

            # blue joint 2
            blue2_1 = np.arctan2(ys[0] - ys[1], mean_zs[0] - mean_zs[1])
            blue2_2 = np.arctan2(ys[1] - ys[2], mean_zs[1] - mean_zs[2]) - blue2_1

            # blue joint 3
            blue3_1 = np.arctan2(xs[0] - ys[1], mean_zs[0] - mean_zs[1])
            blue3_2 = np.arctan2(xs[1] - xs[2], mean_zs[1] - mean_zs[2]) - blue3_1


            print("Predicted: ", [ja1, blue2_2, blue3_2, ja3])

    @staticmethod
    def detect_joint_angle(pos1, pos2):
        mag1 = np.linalg.norm(pos1)
        mag2 = np.linalg.norm(pos2)
        dot = np.dot(pos1, pos2)
        return np.arccos(dot / (mag1 * mag2))


if __name__ == "__main__":
    server = Server()

    rospy.init_node('angles')
    rospy.Subscriber("/joints_pos_y", Float64MultiArray, server.image1_callback_y)
    rospy.Subscriber("/joints_pos_z_1", Float64MultiArray, server.image1_callback_z)
    rospy.Subscriber("/joints_pos_x", Float64MultiArray, server.image2_callback_x)
    rospy.Subscriber("/joints_pos_z_2", Float64MultiArray, server.image2_callback_z)

    rospy.spin()
