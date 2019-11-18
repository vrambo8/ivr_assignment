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
        self.joints_pub2 = rospy.Publisher("joints_pos2", Float64MultiArray, queue_size=10)
        # initialize the bridge between openCV and ROS
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

    def detect_orange(self, image):
        mask = cv2.inRange(image, (57, 100, 120), (99, 190, 227))
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        _, contours, _ = cv2.findContours(img.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        print(len(contours))

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
        return np.array([ja1, ja2, ja3])


        # In this method you can focus on detecting the rotation of link 1

    def detect_l1(self, image, quadrant):
        # find the center of the link
        circle1Pos = self.detect_yellow(image)
        circle2Pos = self.detect_blue(image)
        center = (circle1Pos + circle2Pos) / 2

        # Isolate the region of interest in the thresholded image
        # (select an 160 by 160 window around the center of the link)
        mask = cv2.inRange(image, (0, 0, 0), (1, 1, 1))
        ROI = mask[center[1] - self.target.shape[0] / 2: (center[1] + self.target.shape[0] / 2) + 1,
              center[0] - self.target.shape[1] / 2: (center[0] + self.target.shape[1] / 2) + 1]
        ROI = ROI[0:self.target.shape[0], 0:self.target.shape[1]]  # making sure it has the same size as the template

        # Apply the distance transform
        dist = cv2.distanceTransform(cv2.bitwise_not(ROI), cv2.DIST_L2, 0)

        # rotate the template by small step sizes around the angles that was already estimated from lab 1 and compare
        # it with the cropped image of the link
        sumlist = np.array([])
        step = 1  # degree increment in the search
        rows, cols = self.target.shape
        quadrant = quadrant - 90  # there is  90 degree difference between the robot frame and frame for rotating the
        # template
        angle_iteration = np.arange(quadrant[0], quadrant[1], step)
        for i in angle_iteration:
            # Rotate the template to the desired rotation configuration
            M = cv2.getRotationMatrix2D((cols / 2, rows / 2), i, 1)
            # Apply rotation to the template
            rotatedTemplate = cv2.warpAffine(self.target, M, (cols, rows))
            # Combine the template and region of interest together to obtain only the values that are inside the template
            img = dist * rotatedTemplate
            # Sum the distances and append to the list
            sumlist = np.append(sumlist, np.sum(img))

        # Once all configurations have been searched then select the one with the smallest distance and convert
        # to radians.
        return (angle_iteration[np.argmin(sumlist)] * np.pi) / 180.0

    # Recieve data, process it, and publish
    def callback2(self, data):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Uncomment if you want to save the image
        cv2.imwrite('image_copy.png', self.cv_image2)
        a = self.detect_joint_angles(self.cv_image2)
        im2 = cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)

        self.joints = Float64MultiArray()
        self.joints.data = a

        #self.target =
        self.detect_orange(self.cv_image2)

        # Publish the results
        try:
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.joints_pub2.publish(self.joints)
            print(a)
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
