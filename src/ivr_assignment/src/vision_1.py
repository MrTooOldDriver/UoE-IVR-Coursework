#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class Vision1:

    # Defines publisher and subscriber
    def __init__(self):
        rospy.init_node('vision1', anonymous=True)
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        self.join2_pub = rospy.Publisher("joint_angle_2", Float64, queue_size=10)
        self.join3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.join4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)
        self.red_pub = rospy.Publisher("end_effector", Float64MultiArray, queue_size=10)
        self.cv_image1 = None
        self.cv_image2 = None
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        self.red = [(0, 0, 100), (0, 0, 255)]
        self.yellow = [(0, 100, 100), (0, 255, 255)]
        self.blue = [(100, 0, 0), (255, 0, 0)]
        self.green = [(0, 100, 0), (0, 255, 0)]
        self.pixel2meter_average = 0.0355  # Pre calculate pixel2meter value
        self.last_vector = None

    def callback1(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.cv_image1 is not None and self.cv_image2 is not None:
            self.find_joint_angle()

    def callback2(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.cv_image1 is not None and self.cv_image2 is not None:
            self.find_joint_angle()

    def detect_color_pos(self, cv_image, color):
        mask = cv2.inRange(cv_image, color[0], color[1])
        # print('mask_area={0} {1}'.format(np.sum(mask), color[1]))
        kernel = np.ones((8, 8), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if M['m00'] == 0:
            return np.array([None, None])
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.circle(cv_image, (cx, cy), 10, color[1], -1)
        return np.array([cx, cy])

    def pixel2meter(self, cv_image, last_pos):
        link1_pos = self.detect_color_pos(cv_image, self.green)
        link2_pos = self.detect_color_pos(cv_image, self.yellow)
        dist = np.sum((link1_pos - link2_pos) ** 2)
        dist = 4 / np.sqrt(dist)
        return float(dist)

    def find_color_location(self, cv_image):
        green = self.detect_color_pos(cv_image, self.green)
        yellow = self.detect_color_pos(cv_image, self.yellow)
        blue = self.detect_color_pos(cv_image, self.blue)
        red = self.detect_color_pos(cv_image, self.red)
        if blue[0] is None:
            blue = yellow
        if red[0] is None:
            red = yellow
        green = self.pixel2meter_average * green
        yellow = self.pixel2meter_average * yellow
        blue = self.pixel2meter_average * blue
        red = self.pixel2meter_average * red
        return green, yellow, blue, red

    def calculate_3d_pos(self, im1_center, im2_center, im1_pos, im2_pos):
        # combining the two images to get the 3d coordinates
        pos_in_cam1 = im1_center - im1_pos
        pos_in_cam2 = im2_center - im2_pos
        x = -pos_in_cam2[0]
        y = -pos_in_cam1[0]
        z = (pos_in_cam2[1] + pos_in_cam1[1]) / 2
        return np.array([x, y, z])

    def find_angle_between_2_vector(self, vector1, vector2):
        dot_product = np.dot(vector1, vector2)
        length_vector1 = np.sqrt(np.dot(vector1, vector1))
        length_vector2 = np.sqrt(np.dot(vector2, vector2))
        return np.arccos(dot_product / (length_vector1 * length_vector2))

    def calculate_angle_to_x_y_plane(self, vector):
        [x, y, z] = vector
        xz_plane_projection = [x, 0, z]
        joint2 = self.find_angle_between_2_vector([0, 0, 1], xz_plane_projection)
        if x < 0:
            joint2 = -joint2
        # use joint2 to calculate new xz_plane_projection after joint2 rotation
        joint3_projection_vector = [np.sin(joint2), 0, np.cos(joint2)]
        joint3 = -self.find_angle_between_2_vector(joint3_projection_vector, vector)
        if y < 0:
            joint3 = -joint3
        return joint3, joint2

    def calculate_angle_to_previous_joint(self, previous_joint_vector, current_joint_vector, x_angle, y_angle):
        # Find blue red vector
        previous_to_current_vector = current_joint_vector - previous_joint_vector
        joint4 = self.find_angle_between_2_vector(previous_joint_vector, previous_to_current_vector)
        return joint4

    def find_joint_angle(self):
        # Fing sphere location on each image
        im1_center, im1_yellow, im1_blue, im1_red = self.find_color_location(self.cv_image1)
        im2_center, im2_yellow, im2_blue, im2_red = self.find_color_location(self.cv_image2)
        # Find yellow blue vector
        yellow_blue_vector = self.calculate_3d_pos(im1_yellow, im2_yellow, im1_blue, im2_blue)
        # Calculate joint3 and joint2
        joint3, joint2 = self.calculate_angle_to_x_y_plane(yellow_blue_vector)
        # Find yellow red vector
        yellow_red_vector = self.calculate_3d_pos(im1_yellow, im2_yellow, im1_red, im2_red)
        # Calculate join4
        joint4 = self.calculate_angle_to_previous_joint(yellow_blue_vector, yellow_red_vector, joint3, joint2)
        # Output end effector position for section3
        red_coord = Float64MultiArray()
        red_coord.data = self.calculate_3d_pos(im1_center, im2_center, im1_red, im2_red)
        self.join2_pub.publish(Float64(joint2))
        self.join3_pub.publish(Float64(joint3))
        self.join4_pub.publish(Float64(joint4))
        self.red_pub.publish(red_coord)
        print('joint2={0} joint3={1} joint4={2}'.format(joint2, joint3, joint4))
        # rostopic pub -1 /robot/joint2_position_controller/command std_msgs/Float64 "data: 1.0"
        # rostopic pub -1 /robot/joint3_position_controller/command std_msgs/Float64 "data: 0.0"
        # rostopic pub -1 /robot/joint4_position_controller/command std_msgs/Float64 "data: 0.0"
        # cv2.imwrite('im1.png', self.cv_image1)
        # cv2.imwrite('im2.png', self.cv_image2)


# call the class
def main(args):
    ic = Vision1()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
