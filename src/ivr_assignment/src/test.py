#!/usr/bin/env python3

import numpy
import numpy as np
# import roslib
import sys
# import rospy
import cv2
import numpy as np
from sympy import sin, cos, Matrix
from sympy.abc import alpha, beta, gamma
import zmq.utils.constant_names


# from std_msgs.msg import Float64MultiArray, Float64
# from target import target_publisher


def get_homogeneous_mat(q):

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

    # Rotation on z axis
    # Rotation on x axis
    # Rotation on y axis
    # Transform to the end effector

    H = As[0]
    for i in range(1, len(As)):
        H = np.matmul(H, As[i])

    return H


# For testing purposes
if __name__ == '__main__':
    q1 = [0.32, 1.234, 0.123]
    H = get_homogeneous_mat(q1)
    print(H)