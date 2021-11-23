import numpy as np
# import roslib
# import sys
# import rospy
import cv2
import numpy as np
import zmq.utils.constant_names
# from std_msgs.msg import Float64MultiArray, Float64
#
# # initialize a publisher for end effector target positions
# joint2_cmd_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
# joint3_cmd_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
# joint4_cmd_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)


# class Q:
#     theta = 0
#     alpha = 0
#     a = 0
#     d = 0


def homogeneous_mat(q):

    As = []

    # Rotation on z axis
    As.append(np.array(
        [[np.cos(q[0]), -np.sin(q[0]), 0, 0],
         [np.sin(q[0]), np.cos(q[0]), 0, 0],
         [0, 0, 1, 4],
         [0, 0, 0, 1]]
    ))
    # Rotation on x axis
    As.append(np.array(
        [[1, 0, 0, 0],
         [0, np.cos(q[2]), -np.sin(q[2]), 0],
         [0, np.sin(q[2]), np.cos(q[2]), 3.2],
         [0, 0, 0, 1]]
    ))
    # Rotation on y axis
    As.append(np.array(
        [[np.cos(q[3]), 0, np.sin(q[3]), 0],
         [0, 1, 0, 0],
         [-np.sin(q[3]), 0, np.cos(q[3]), 2.8],
         [0, 0, 0, 1]]
    ))

    H = As[0]
    for i in range(1, len(As)):
        H = np.matmul(H, As[i])

    return H



# For testing purposes
if __name__ == '__main__':
    q = [1, 1, 1, 1]
    H = homogeneous_mat(q)
    print(H)

