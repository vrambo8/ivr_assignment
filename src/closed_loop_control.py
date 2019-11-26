#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import sympy as sym
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from inverse_kinematics import *


class Server:
    def __init__(self):
        self.zs1 = None
        self.ys = None
        self.zs2 = None
        self.xs = None
	self.target_y = None
	self.target_z1 = None
	self.target_x = None
	self.target_z2 = None

        rospy.init_node('closed_loop_control', anonymous=True)
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
	self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # initialize the bridge between openCV and ROS
	self.end_x = rospy.Publisher("end_x", Float64, queue_size=10)
	self.end_y = rospy.Publisher("end_y", Float64, queue_size=10)
	self.end_z = rospy.Publisher("end_z", Float64, queue_size=10)

        self.bridge = CvBridge()
        # record the begining time
        self.time_trajectory = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')


    def callback(self, ys, zs1, xs, zs2, target_y, target_z1, target_x, target_z2):
        
	self.ys = ys
        self.zs1 = zs1
        self.xs = xs
        self.zs2 = zs2
	self.target_y = target_y
	self.target_z1 = target_z1
	self.target_x = target_x
	self.target_z2 = target_z2
        
   
        # send control commands to joints (lab 3)
        q_d = self.closed_loop()
	#if q_d is None: return
	while q_d[0] > np.pi/2:
	    q_d[0] -= np.pi
	while q_d[0] < -np.pi/2:
	    q_d[0] += np.pi

	q_d[1] = min(q_d[1], np.pi)
	q_d[1] = max(q_d[1], -np.pi)
	q_d[2] = min(q_d[2], np.pi)
	q_d[2] = max(q_d[2], -np.pi)
	q_d[3] = min(q_d[3], np.pi)
	q_d[3] = max(q_d[3], -np.pi)
        
        self.joint1 = Float64()
        self.joint1.data = q_d[0]
        self.joint2 = Float64()
        self.joint2.data = q_d[1]
        self.joint3 = Float64()
        self.joint3.data = -q_d[2]
	self.joint4 = Float64()
        self.joint4.data = q_d[3]

        # Publishing the desired trajectory on a topic named trajectory(lab 3)
        x_d = self.trajectory()  # getting the desired trajectory
        self.trajectory_desired = Float64MultiArray()
        self.trajectory_desired.data = x_d

        # Publish the results
        try:
            self.trajectory_pub.publish(self.trajectory_desired)
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
	    self.robot_joint4_pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)

    def closed_loop(self):
        if self.zs1 is not None and self.ys is not None and self.zs2 is not None and self.xs is not None and \
		self.target_z1 is not None and self.target_z2 is not None and self.target_x is not None and self.target_y is not None:
          
            mean_zs = [np.mean([z1, z2]) for (z1, z2) in zip(self.zs1.data, self.zs2.data)]
            xs = self.xs.data
            ys = self.ys.data
            zs1 = self.zs1.data
            zs2 = self.zs2.data	    

	    mean_target_z = np.mean([self.target_z1.data, self.target_z2.data])
            target_x = self.target_x.data
            target_y = self.target_y.data
	

            # joint positions
	    mean_z_effector = np.mean([zs1[0] - zs1[3], zs2[0] - zs2[3]])
            effector_pos = np.array([xs[3] - xs[0], ys[0] - ys[3], mean_z_effector])
	    mean_z_green = np.mean([zs1[0] - zs1[2], zs2[0] - zs2[2]])
            green_pos = np.array([xs[2] - xs[0], ys[0] - ys[2], mean_z_green])

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
            pos_d = np.array([target_x, target_y, mean_target_z])
	    print(pos_d, " Target")
            # estimate derivative of error

            self.error_d = ((pos_d - pos) - self.error) / dt
            # estimate error
            self.error = pos_d - pos
	   
	    self.end_x.publish(pos[0])
	    self.end_y.publish(pos[1])
	    self.end_z.publish(pos[2])
	    print(pos, " Effector")

            q = solve_angles(pos, green_pos) # estimate initial value of joints'
	    
            J_inv = np.linalg.pinv(self.jacobian(q))
	     # calculating the pseudo inverse of Jacobian
            dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose())))  # control input (angular velocity of joints)
            q_d = q + (dt * dq_d)  # control input (angular position of joints)
            return [q for q in q_d]

    def trajectory(self):
	cur_time = np.array([rospy.get_time() - self.time_trajectory])
	x_d = float(6* np.cos(cur_time * np.pi/100))
	y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
	return np.array([x_d, y_d, 0])

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
        return np.array(J.evalf(), dtype='float64')

    def kinematic_eqs(self, joint_angles):
        theta = joint_angles

        DH = [[theta[0], - sym.pi / 2, 2, 0], [theta[1], sym.pi / 2, 0, 0],
              [theta[2], -sym.pi / 2, 0, 3], [theta[3], 0, 0, 2]]

        initial = sym.Matrix([0, 0, 0, 1])
        final_matrix = self.create_trans_matrix(DH[0])

        for row in DH[1:]:
            new_mat = self.create_trans_matrix(row)
            final_matrix = final_matrix * new_mat

        eqs = sym.Matrix((final_matrix*(initial))[:3])

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

    ys = message_filters.Subscriber("/joints_pos_y", Float64MultiArray)
    zs1 = message_filters.Subscriber("/joints_pos_z_1", Float64MultiArray)
    xs = message_filters.Subscriber("/joints_pos_x", Float64MultiArray)
    zs2 = message_filters.Subscriber("/joints_pos_z_2", Float64MultiArray)
    target_y = message_filters.Subscriber("/target_y", Float64)
    target_z1 = message_filters.Subscriber("/target_z_1", Float64)
    target_x = message_filters.Subscriber("/target_x", Float64)
    target_z2 = message_filters.Subscriber("/target_z_2", Float64)

    ts = message_filters.ApproximateTimeSynchronizer([ys, zs1, xs, zs2, target_y, target_z1, target_x, target_z2], 1, 0.1, allow_headerless=True)
    ts.registerCallback(server.callback)

    rospy.spin()

    print(server.closed_loop())
