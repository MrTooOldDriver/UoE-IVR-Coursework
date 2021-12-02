#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64


def sinusoidal_signals():

    # Defines publisher and subscriber
    # initialize the node named
    rospy.init_node('sinusoidal_signals', anonymous=True)
    rate = rospy.Rate(50)  # 50hz
    # initialize a publisher for end effector target positions
    joint1_cmd_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    joint2_cmd_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    joint3_cmd_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    joint4_cmd_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        magic_value = 0.5
        current_time = np.array([rospy.get_time()]) - t0
        joint1_data = np.sin(current_time * (np.pi / 28))
        joint1_data = joint1_data * np.pi
        joint2_data = np.sin(current_time * (np.pi / 15))
        joint2_data = joint2_data * magic_value * np.pi
        joint3_data = np.sin(current_time * (np.pi / 20))
        joint3_data = joint3_data * magic_value * np.pi
        joint4_data = np.sin(current_time * (np.pi / 18))
        joint4_data = joint4_data * magic_value * np.pi
        joint1_cmd_pub.publish(Float64(joint1_data))
        # joint2_cmd_pub.publish(Float64(joint2_data))
        joint3_cmd_pub.publish(Float64(joint3_data))
        joint4_cmd_pub.publish(Float64(joint4_data))
        print('joint1_data:{0} joint2_data:{1} joint3_data:{2} joint4_data:{3}'
              .format(joint1_data, joint2_data, joint3_data, joint4_data))
        rate.sleep()


# run the code if the node is called
if __name__ == '__main__':
    try:
        sinusoidal_signals()
    except rospy.ROSInterruptException:
        pass
