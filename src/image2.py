#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub_x = rospy.Publisher("joints_pos_x", Float64MultiArray, queue_size=10)
        self.joints_pub_z_2 = rospy.Publisher("joints_pos_z_2", Float64MultiArray, queue_size=10)
        # initialize a publisher to send target z, x position
        self.target_x = rospy.Publisher("target_x", Float64, queue_size=10)
        self.target_z_2 = rospy.Publisher("target_z_2", Float64, queue_size=10)
        self.bridge = CvBridge()

    def detect_red(self, image):
        # Isolate the blue colour in the image as a binary image
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(mask)
        # Calculate pixel coordinates for the centre of the blob
        try:
            cx = int(M['m10'] / M['m00'])
        except ZeroDivisionError:
            return None
        try:
            cy = int(M['m01'] / M['m00'])
        except ZeroDivisionError:
            return None
        return np.array([cx, cy])

        # Detecting the centre of the green circle

    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        try:
            cx = int(M['m10'] / M['m00'])
        except ZeroDivisionError:
            return None
        try:
            cy = int(M['m01'] / M['m00'])
        except ZeroDivisionError:
            return None
        return np.array([cx, cy])

        # Detecting the centre of the blue circle

    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        try:
            cx = int(M['m10'] / M['m00'])
        except ZeroDivisionError:
            return None
        try:
            cy = int(M['m01'] / M['m00'])
        except ZeroDivisionError:
            return None
        return np.array([cx, cy])

        # Detecting the centre of the yellow circle

    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # cv2.imshow('mask2', mask)
        M = cv2.moments(mask)
        try:
            cx = int(M['m10'] / M['m00'])
        except ZeroDivisionError:
            return None
        try:
            cy = int(M['m01'] / M['m00'])
        except ZeroDivisionError:
            return None

        return np.array([cx, cy])

    def detect_target(self, image):
        mask = cv2.inRange(image, (57, 100, 120), (99, 190, 227))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # cv2.imshow('mask', mask)
        _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        maxArea = 0
        for c in contours:
            if (cv2.contourArea(c) > maxArea):
                maxArea = cv2.contourArea(c)
                sphere = c
        M = cv2.moments(sphere)
        try:
            cx = int(M['m10'] / M['m00'])
        except ZeroDivisionError:
            return None
        try:
            cy = int(M['m01'] / M['m00'])
        except ZeroDivisionError:
            return None

        a = self.pixel2meter(image)
        center = a * self.detect_yellow(image)
        target = a * np.array([cx, cy])

        dist = np.abs([target[0] - center[0], target[1] - center[1]])
        return dist

    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_blue(image)
        circle2Pos = self.detect_green(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 3 / np.sqrt(dist)

    # Calculate the relevant joint angles from the image
    def detect_joint_angles(self, image):
        a = self.pixel2meter(image)
        # Obtain the centre of each coloured blob
        center = a * self.detect_yellow(image)
        circle1Pos = a * self.detect_blue(image)
        circle2Pos = a * self.detect_green(image)
        circle3Pos = a * self.detect_red(image)

        # Solve using trigonometry
        ja1 = np.arctan2(center[0] - circle1Pos[0], center[1] - circle1Pos[1])
        ja2 = np.arctan2(circle1Pos[0] - circle2Pos[0], circle1Pos[1] - circle2Pos[1]) - ja1
        ja3 = np.arctan2(circle2Pos[0] - circle3Pos[0], circle2Pos[1] - circle3Pos[1]) - ja2 - ja1
        # return np.array([ja1, ja2, ja3])
        return np.array([center[0], circle1Pos[0], circle2Pos[0], circle3Pos[0]]), np.array(
            [center[1], circle1Pos[1], circle2Pos[1], circle3Pos[1]])

    # Recieve data, process it, and publish
    def callback2(self, data):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Uncomment if you want to save the image

        xs, zs = self.detect_joint_angles(self.cv_image2)
        im2 = cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)

        self.joints_x = Float64MultiArray()
        self.joints_x.data = xs

        self.joints_z = Float64MultiArray()
        self.joints_z.data = zs

        target = self.detect_target(self.cv_image2)
        self.x = Float64()
        self.x.data = target[0]
        # print(self.x.data)

        self.z = Float64()
        self.z.data = target[1]
        # Publish the results
        try:
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.joints_pub_x.publish(self.joints_x)
            self.joints_pub_z_2.publish(self.joints_z)
            self.target_x.publish(self.x)
            self.target_z_2.publish(self.z)
            # print(a)
        except CvBridgeError as e:
            print(e)


# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
