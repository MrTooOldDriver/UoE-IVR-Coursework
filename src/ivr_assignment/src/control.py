#!/usr/bin/env python3

import rospy
import numpy as np
from sympy import sin, cos, Matrix
from sympy.abc import alpha, beta, gamma
from std_msgs.msg import Float64MultiArray, Float64


def get_homogeneous_mat_q21(q):

    # This function was written for question 2.1

    # Rotation on y axis
    # Rotation on x axis
    # Transform to the blue

    As = [np.array(
        [[np.cos(q[0]), 0, np.sin(q[0]), 0], [0, 1, 0, 0],
         [-np.sin(q[0]), 0, np.cos(q[0]), 0], [0, 0, 0, 1]]
    ), np.array(
        [[1, 0, 0, 0], [0, np.cos(q[1]), -np.sin(q[1]), 0],
         [0, np.sin(q[1]), np.cos(q[1]), 0], [0, 0, 0, 1]]
    ), np.array(
        [[1, 0, 0, 0],
         [0, 1, 0, 0],
         [0, 0, 1, 3.2],
         [0, 0, 0, 1]]
    )]

    H = As[0]
    for i in range(1, len(As)):
        H = np.matmul(H, As[i])

    return H


def get_homogeneous_mat(q):

    # Rotation on z axis
    # Rotation on x axis
    # Rotation on y axis
    # Transform to the end effector

    As = [np.array(
        [[np.cos(q[0]), -np.sin(q[0]), 0, 0], [np.sin(q[0]), np.cos(q[0]), 0, 0],
         [0, 0, 1, 0], [0, 0, 0, 1]]
    ), np.array(
        [[1, 0, 0, 0], [0, np.cos(q[1]), -np.sin(q[1]), 0],
         [0, np.sin(q[1]), np.cos(q[1]), 4], [0, 0, 0, 1]]
    ), np.array(
        [[np.cos(q[2]), 0, np.sin(q[2]), 0], [0, 1, 0, 0],
         [-np.sin(q[2]), 0, np.cos(q[2]), 3.2], [0, 0, 0, 1]]
    ), np.array(
        [[1, 0, 0, 0],
         [0, 1, 0, 0],
         [0, 0, 1, 2.8],
         [0, 0, 0, 1]]
    )]

    H = As[0]
    for i in range(1, len(As)):
        H = np.matmul(H, As[i])

    return H


def get_jacobian(q):

    # The 3-d location coordinate vector from homogeneous
    O = Matrix(
        [3.2 * sin(alpha) * sin(beta) + 2.8 * (cos(alpha) * sin(gamma) + sin(alpha) * sin(beta) * cos(gamma)),
         2.8 * (sin(alpha) * sin(gamma) - cos(alpha) * sin(beta) * cos(gamma)) - 3.2 * cos(alpha) * sin(beta),
         2.8 * cos(beta) * cos(gamma) + 3.2 * cos(beta) + 4])

    # Get jacobian matirx using sympy
    J = O.jacobian(Matrix([alpha, beta, gamma]))
    return np.matrix(J.subs([(alpha, q[0]), (beta, q[1]), (gamma, q[2])]).evalf(), dtype='float')


def get_end_effector_pos(q):
    return get_homogeneous_mat(q)[:3, 3]


def get_pseudo_inverse(J):
    return np.matmul(J.T, np.linalg.inv(np.matmul(J, J.T)))


def get_target_angles(q, err, dt):
    J = get_jacobian(q)
    J_inv = get_pseudo_inverse(J)
    q_d = q + (dt * np.dot(J_inv, err.T))
    return q_d


class Control:

    def __init__(self):

        ####################################################
        #################  QUESTION SWITCH #################
        ####################################################

        # The arg to select between Q1 (FK/False) and Q2 (IK/True)
        self.OPEN_LOOP = True

        ####################################################
        #################  QUESTION SWITCH #################
        ####################################################

        # Defines publisher and subscriber
        # initialize the node named
        rospy.init_node('control', anonymous=True)
        self.rate = rospy.Rate(50)  # 50hz
        self.t1 = rospy.get_time()

        self.joint1 = 0
        self.joint3 = 0
        self.joint4 = 0
        self.end_effector = np.array([0, 0, 0])
        self.target_pos = np.array([0, 0, 0])

        # initialize publishers

        self.target_pos_sub = rospy.Subscriber("target_pos", Float64MultiArray, self.target_pos_callback)

        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.true_ef_x_pub = rospy.Publisher("true_ef_x", Float64, queue_size=10)
        self.true_ef_y_pub = rospy.Publisher("true_ef_y", Float64, queue_size=10)
        self.true_ef_z_pub = rospy.Publisher("true_ef_z", Float64, queue_size=10)
        self.tar_ef_x_pub = rospy.Publisher("tar_ef_x", Float64, queue_size=10)
        self.tar_ef_y_pub = rospy.Publisher("tar_ef_y", Float64, queue_size=10)
        self.tar_ef_z_pub = rospy.Publisher("tar_ef_z", Float64, queue_size=10)

        self.joint1_sub = rospy.Subscriber("joint_angle_1", Float64, self.joint_1_callback)
        self.joint3_sub = rospy.Subscriber("joint_angle_3", Float64, self.joint_3_callback)
        self.joint4_sub = rospy.Subscriber("joint_angle_4", Float64, self.joint_4_callback)
        
        self.end_effector_sub = rospy.Subscriber("end_effector", Float64MultiArray, self.end_effector_callback)

    def forward_kinematics(self):

        # To observe & verify that FK is working properly

        # Use angles read from topics
        q = [self.joint1, self.joint3, self.joint4]

        # Or set different angles manually (output 10 sets by hand)
        q = [0.6, 0.7, 0.9]

        self.robot_joint1_pub.publish(Float64(q[0]))
        self.robot_joint3_pub.publish(Float64(q[1]))
        self.robot_joint4_pub.publish(Float64(q[2]))

        cur_pos = get_end_effector_pos(q)

        # print("Joints 1, 3, 4: " + str(q))
        # print("Calculated by FK: " + str(cur_pos))
        # print("Estimation from image: " + str(self.end_effector))

    def inverse_kinematics(self):
        try:

            cur_time = rospy.get_time()
            dt = cur_time - self.t1

            q = [self.joint1, self.joint3, self.joint4]
            cur_pos = get_end_effector_pos(q)

            target_pos = np.array(self.target_pos)
            err = target_pos - cur_pos
            q_d = get_target_angles(q, err, dt)

            print("Error from target: " + str(err))
            q_d = np.squeeze(np.asarray(q_d))

            self.robot_joint1_pub.publish(Float64(q_d[0]))
            self.robot_joint3_pub.publish(Float64(q_d[1]))
            self.robot_joint4_pub.publish(Float64(q_d[2]))

            self.true_ef_x_pub.publish(Float64(cur_pos[0]))
            self.tar_ef_x_pub.publish(Float64(target_pos[0]))
            self.true_ef_y_pub.publish(Float64(cur_pos[1]))
            self.tar_ef_y_pub.publish(Float64(target_pos[1]))
            self.true_ef_z_pub.publish(Float64(cur_pos[2]))
            self.tar_ef_z_pub.publish(Float64(target_pos[2]))

            self.t1 = cur_time

            for i in range(2):
                self.rate.sleep()

        except:
            pass

    def control_main(self):
        # Switch between Q1 (FK) and Q2 (IK)
        if self.OPEN_LOOP:
            self.inverse_kinematics()
        else:
            self.forward_kinematics()

    def joint_1_callback(self, data):
        self.joint1 = data.data
        self.control_main()

    def joint_3_callback(self, data):
        self.joint3 = data.data
        self.control_main()

    def joint_4_callback(self, data):
        self.joint4 = data.data
        self.control_main()

    def end_effector_callback(self, data):
        self.end_effector = np.array(data.data)
        self.control_main()

    def target_pos_callback(self, data):
        self.target_pos = np.array(data.data)
        self.control_main()


# run the code if the node is called
if __name__ == '__main__':
    ic = Control()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down control")


