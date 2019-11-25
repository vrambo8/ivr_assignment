#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import sympy as sym
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

from inverse_kinematics import *


class Server:
    def __init__(self):
        self.zs1 = None
        self.ys = None
        self.zs2 = None
        self.xs = None

        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send messages to a topic named image_topic
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
        # initialize a publisher to send robot end-effector position
        self.end_effector_pub = rospy.Publisher("end_effector_prediction", Float64MultiArray, queue_size=10)
        # initialize a publisher to send desired trajectory
        self.trajectory_pub = rospy.Publisher("trajectory", Float64MultiArray, queue_size=10)
        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub = rospy.Subscriber("/robot/camera1/image_raw", Image, self.callback)
        # record the begining time
        self.time_trajectory = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0], dtype='float64')

    def image1_callback_y(self, msg):
        self.ys = msg
        # self.calculate_angles()

    def image1_callback_z(self, msg):
        self.zs1 = msg
        # self.calculate_angles()

    def image2_callback_x(self, msg):
        self.xs = msg
        # self.calculate_angles()

    def image2_callback_z(self, msg):
        self.zs2 = msg
        # self.calculate_angles()

    def calculate_angles(self):
        if self.zs1 is not None and self.ys is not None and self.zs2 is not None and self.xs is not None:
            # print("Image1: ", zip(self.ys.data, self.zs1.data))
            # print("Image2: ", zip(self.xs.data, self.zs2.data))
            mean_zs = [np.mean([z1, z2]) for (z1, z2) in zip(self.zs1.data, self.zs2.data)]
            xs = self.xs.data
            ys = self.ys.data

            # joint positions
            effector_pos = np.array([xs[0], ys[0], mean_zs[0]]) - np.array([xs[3], ys[3], mean_zs[3]])
            print("Current Position: ", effector_pos)

            # P gain
            K_p = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
            # D gain
            K_d = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
            # estimate time step
            cur_time = np.array([rospy.get_time()])
            dt = cur_time - self.time_previous_step
            self.time_previous_step = cur_time
            # robot end-effector position
            pos = effector_pos
            # desired trajectory
            pos_d = self.trajectory()
            # estimate derivative of error
            self.error_d = ((pos_d - pos) - self.error) / dt
            # estimate error
            self.error = pos_d - pos
            q = solve_angles(effector_pos) # estimate initial value of joints'
            J_inv = np.linalg.pinv(self.calculate_jacobian(q))  # calculating the psudeo inverse of Jacobian
            dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose())))  # control input (angular velocity of joints)
            q_d = q + (dt * dq_d)  # control input (angular position of joints)
            return q_d

    def jacobian(self, joint_angles):
        t1 = sym.symbols('t1')
        t2 = sym.symbols('t2')
        t3 = sym.symbols('t3')
        t4 = sym.symbols('t4')
        t = joint_angles
        eqs = self.kinematic_eqs(sym.Array([t1, t2, t3, t4]))
        row1 = [sym.diff(eqs[0], t1), sym.diff(eqs[0], t2), sym.diff(eqs[0], t3), sym.diff(eqs[0], t4)]
        row2 = [sym.diff(eqs[1], t1), sym.diff(eqs[1], t2), sym.diff(eqs[1], t3), sym.diff(eqs[1], t4)]
        row3 = [sym.diff(eqs[2], t1), sym.diff(eqs[2], t2), sym.diff(eqs[2], t3), sym.diff(eqs[2], t4)]
        J = (((sym.Matrix([row1, row2, row3]).subs(t1, t[0])).subs(t2, t[1])).subs(t3, t[2])).subs(t4, t[3])
        return np.array(J.evalf())

    def kinematic_eqs(self, joint_angles):
        theta = joint_angles

        DH = [[theta[0], - sym.pi / 2, 2, 0], [theta[1], sym.pi / 2, 0, 0],
              [theta[2], -sym.pi / 2, 0, 3], [theta[3], 0, 0, 2]]

        initial = sym.Matrix([0, 0, 0, 1])
        final_matrix = self.create_trans_matrix(DH[0])

        for row in DH[1:]:
            new_mat = self.create_trans_matrix(row)
            final_matrix = final_matrix * new_mat

        eqs = sym.Matrix(final_matrix.dot(initial)[:3])

        return sym.expand(eqs)

    @staticmethod
    def create_trans_matrix(row):
        theta = row[0]
        alpha = row[1]
        d = row[2]
        r = row[3]

        row1 = [sym.cos(theta), sym.sin(theta) * sym.cos(alpha), sym.sin(theta) * sym.sin(alpha),
                r * sym.cos(theta)]
        row2 = [-sym.sin(theta), sym.cos(theta) * sym.cos(alpha), sym.cos(theta) * sym.sin(alpha),
                -r * sym.sin(theta)]
        row3 = [0, -sym.sin(alpha), sym.cos(alpha), d]
        row4 = [0, 0, 0, 1]

        M = sym.Matrix([row1, row2, row3, row4])
        return M


if __name__ == "__main__":
    server = Server()

    rospy.init_node('angles')
    rospy.Subscriber("/joints_pos_y", Float64MultiArray, server.image1_callback_y)
    rospy.Subscriber("/joints_pos_z_1", Float64MultiArray, server.image1_callback_z)
    rospy.Subscriber("/joints_pos_x", Float64MultiArray, server.image2_callback_x)
    rospy.Subscriber("/joints_pos_z_2", Float64MultiArray, server.image2_callback_z)

    rospy.spin()

    server.calculate_angles()
