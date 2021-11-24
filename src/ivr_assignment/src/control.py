#!/usr/bin/env python3

import numpy
import numpy as np
import roslib
import sys
import rospy
import cv2
import numpy as np
from sympy import sin, cos, Matrix
from sympy.abc import alpha, beta, gamma
import zmq.utils.constant_names
from std_msgs.msg import Float64MultiArray, Float64
from target import target_publisher


# # initialize a publisher for end effector target positions
# joint2_cmd_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
# joint3_cmd_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
# joint4_cmd_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)


# class Q:
#     theta = 0
#     alpha = 0
#     a = 0
#     d = 0

def open_loop():

    # Defines publisher and subscriber
    # initialize the node named
    rospy.init_node('target_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50hz

    # initialize publishers and subscribers for end effector target positions
    target_pos_pub = rospy.Subscriber("target_pos", Float64MultiArray, target_publisher)
    # angle_1 = rospy.Subscriber("joint_angle_1", Float64MultiArray, )
    # angle_3 = rospy.Subscriber("joint_angle_3", Float64MultiArray, )
    # angle_4 = rospy.Subscriber("joint_angle_4", Float64MultiArray, )

    target_pos_pub = rospy.Subscriber("target_pos", Float64MultiArray, target_publisher)
    target_pos_pub = rospy.Subscriber("target_pos", Float64MultiArray, target_publisher)

    robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)

    t0 = rospy.get_time()


def get_homogeneous_mat(q):

    As = [np.array(
        [[np.cos(q[0]), -np.sin(q[0]), 0, 0], [np.sin(q[0]), np.cos(q[0]), 0, 0],
         [0, 0, 1, 0], [0, 0, 0, 1]]
    ), np.array(
        [[1, 0, 0, 0], [0, np.cos(q[2]), -np.sin(q[2]), 0],
         [0, np.sin(q[2]), np.cos(q[2]), 4], [0, 0, 0, 1]]
    ), np.array(
        [[np.cos(q[3]), 0, np.sin(q[3]), 0], [0, 1, 0, 0],
         [-np.sin(q[3]), 0, np.cos(q[3]), 3.2], [0, 0, 0, 1]]
    ), np.array(
        [[1, 0, 0, 0],
         [0, 1, 0, 0],
         [0, 0, 1, 2.8],
         [0, 0, 0, 1]]
    )]

    # Rotation on z axis
    # Rotation on x axis
    # Rotation on y axis
    # Transform to the end effector

    H = As[0]
    for i in range(1, len(As)):
        H = np.matmul(H, As[i])

    return H


def get_jacobian(q):
    # H = Matrix([[cos(alpha) * cos(gamma) - sin(alpha) * sin(beta) * sin(gamma), -sin(alpha) * cos(beta),
    #              cos(alpha) * sin(gamma) + sin(alpha) * sin(beta) * cos(gamma), 2.8 * sin(alpha) * sin(beta)],
    #             [sin(alpha) * cos(gamma) + cos(alpha) * sin(beta) * sin(gamma), cos(alpha) * cos(beta),
    #              sin(alpha) * sin(gamma) - cos(alpha) * sin(beta) * cos(gamma), -2.8 * cos(alpha) * sin(beta)],
    #             [-cos(beta) * sin(gamma), sin(beta), cos(beta) * cos(gamma), 2.8 * cos(beta) + 7.2],
    #             [0, 0, 0, 1]])
    H = Matrix([3.2 * sin(alpha) * sin(beta) + 2.8 * (cos(alpha) * sin(gamma) + sin(alpha) * sin(beta) * cos(gamma)),
                2.8 * (sin(alpha) * sin(gamma) - cos(alpha) * sin(beta) * cos(gamma)) - 3.2 * cos(alpha) * sin(beta),
                2.8 * cos(beta) * cos(gamma) + 3.2 * cos(beta) + 4])
    J = H.jacobian(Matrix([alpha, beta, gamma]))
    return np.matrix(J.subs([(alpha, q[0]), (beta, q[2]), (gamma, q[3])]).evalf(), dtype='float')


def get_pseudo_inverse(J):
    return np.linalg.inv(J)


def get_ik_angles(q, err, dt):
    J = get_jacobian(q)
    J_inv = get_pseudo_inverse(J)
    q_d = q + (dt * np.dot(J_inv, err.T))
    return q_d


# run the code if the node is called
if __name__ == '__main__':
    try:
        open_loop()
    except rospy.ROSInterruptException:
        pass



# # For testing purposes
# if __name__ == '__main__':
#     q1 = [1, 1, 1]
#     H = get_homogeneous_mat(q1)
#     print(H[:3, 3])
#     print(get_jacobian(q1))
#     J_p = get_ik_angles(q1, H[:3, 3])
#     print(J_p)
