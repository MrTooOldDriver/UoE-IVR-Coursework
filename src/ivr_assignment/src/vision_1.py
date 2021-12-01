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
        # rostopic pub -l /robot/joint2_position_controller/command std_msgs/Float64 "data: 1.0"
        # initialize the node
        rospy.init_node('vision1', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        # self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        self.join2_pub = rospy.Publisher("join2", Float64, queue_size=10)
        self.join3_pub = rospy.Publisher("join3", Float64, queue_size=10)
        self.join4_pub = rospy.Publisher("join4", Float64, queue_size=10)
        self.red_pub = rospy.Publisher("red", Float64MultiArray, queue_size=10)
        self.cv_image1 = None
        self.cv_image2 = None
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        self.red = [(0, 0, 100), (0, 0, 255)]
        self.yellow = [(0, 100, 100), (0, 255, 255)]
        self.blue = [(100, 0, 0), (255, 0, 0)]
        self.green = [(0, 100, 0), (0, 255, 0)]
        self.last_pos_1 = [[0, 0], [0, 0], [0, 0], [0, 0]]
        self.last_pos_2 = [[0, 0], [0, 0], [0, 0], [0, 0]]
        self.pixel2meter_average = 0.0355

    # Recieve data from camera 1, process it, and publish
    def callback1(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # self.angle_3_4 = self.calculate_3_4_angle(self.cv_image1)
        # print('Angle2={0}, Angle3={1}'.format(self.angle_2_4, self.angle_3_4))
        if self.cv_image1 is not None and self.cv_image2 is not None:
            # self.calculate_angle()
            self.test_function()

    # Recieve data from camera 2, process it, and publish
    def callback2(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # self.angle_2_4 = self.calculate_2_4_angle(self.cv_image2)
        # print('Angle2={0}, Angle3={1}'.format(self.angle_2_4, self.angle_3_4))
        if self.cv_image1 is not None and self.cv_image2 is not None:
            # self.calculate_angle()
            self.test_function()

    def detect_color_pos(self, cv_image, color, pervious_pos):
        mask = cv2.inRange(cv_image, color[0], color[1])
        # print('mask_area={0} {1}'.format(np.sum(mask), color[1]))
        kernel = np.ones((8, 8), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if M['m00'] == 0:
            print(pervious_pos)
            return np.array([None, None])
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.circle(cv_image, (cx, cy), 10, color[1], -1)
        return np.array([cx, cy])

    def pixel2meter(self, cv_image, last_pos):
        # Obtain the centre of each coloured blob
        link1_pos = self.detect_color_pos(cv_image, self.green, last_pos[0])
        link2_pos = self.detect_color_pos(cv_image, self.yellow, last_pos[1])
        # find the distance between two circles
        dist = np.sum((link1_pos - link2_pos) ** 2)
        # print('Distance={0}'.format(4 / np.sqrt(dist)))
        dist = 4 / np.sqrt(dist)
        return float(dist)

    def find_color_location(self, cv_image, a, last_pos):
        green = a * self.detect_color_pos(cv_image, self.green, last_pos[0])
        yellow = a * self.detect_color_pos(cv_image, self.yellow, last_pos[1])
        blue = a * self.detect_color_pos(cv_image, self.blue, last_pos[2])
        red = a * self.detect_color_pos(cv_image, self.red, last_pos[3])
        return green, yellow, blue, red

    def calculate_angle_with_2image(self, angle_image1, angle_image2, link_length):
        image1_opposite = np.sin(angle_image1) * link_length
        image2_opposite = np.sin(angle_image2) * link_length
        bevel_adjacent = np.sqrt(image1_opposite ** 2 + image2_opposite ** 2)
        return (np.pi / 2) - np.arccos(bevel_adjacent / link_length)

    def find_angle(self, pos1, pos2):
        # result = np.arctan((pos1[0] - pos2[0]) / (pos1[1] - pos2[1]))
        result = np.arctan2(pos1[0] - pos2[0], pos1[1] - pos2[1])
        # if result > (np.pi / 2):
        #     result = result - np.pi
        # elif result < (-np.pi / 2):
        #     result = result + np.pi
        return result

    def adjustment(self, pos1, pos2, link_length):
        dist = np.sum((pos1 - pos2) ** 2)
        dist = np.sqrt(dist)
        if dist > link_length:
            return 0
        else:
            return np.arccos(dist / link_length)

    # def calculate_angle(self):
    #     center, yellow, blue, red = self.find_color_location(self.cv_image1, self.pixel2meter_average, self.last_pos_1)
    #     # joint3 = np.arctan2(yellow[0] - blue[0], yellow[1] - blue[1])
    #     joint3 = self.find_angle(yellow, blue)
    #     # im1_angle_4_5 = np.arctan2(blue[0] - red[0], blue[1] - red[1]) - joint3
    #     self.last_pos_1 = [center, yellow, blue, red]
    #     # Calculate the angle between node from second camera
    #     center, yellow, blue, red = self.find_color_location(self.cv_image2, self.pixel2meter_average, self.last_pos_2)
    #     joint2 = self.find_angle(yellow, blue)
    #     im2_angle_4_5 = self.find_angle(blue, red) - joint2
    #     self.last_pos_2 = [center, yellow, blue, red]
    #     # actual_angle_4_5 = self.calculate_angle_with_2image(im1_angle_4_5, im2_angle_4_5, 2.8)
    #     self.join2_pub.publish(Float64(joint2))
    #     self.join3_pub.publish(Float64(joint3))
    #     self.join4_pub.publish(Float64(im2_angle_4_5))
    #     print('joint2={0} joint3={1} joint4={2}'.format(joint2, joint3, im2_angle_4_5))
    #     cv2.imwrite('im1.png', self.cv_image1)
    #     cv2.imwrite('im2.png', self.cv_image2)

    def find_distance(self, pos1, pos2):
        dist = np.sum((pos1 - pos2) ** 2)
        dist = np.sqrt(dist)
        return dist

    def calculate_angle(self):
        im1_center, im1_yellow, im1_blue, im1_red = self.find_color_location(self.cv_image1, self.pixel2meter_average,
                                                                             self.last_pos_1)
        im2_center, im2_yellow, im2_blue, im2_red = self.find_color_location(self.cv_image2, self.pixel2meter_average,
                                                                             self.last_pos_2)

        # print(np.sqrt(np.sum((im2_yellow - im2_blue) ** 2)))
        # print('adjust_cv2={0}'.format(self.adjustment(im2_yellow, im2_blue, 3.2)))
        # print('joint2={0}'.format(joint2))

        joint2 = -self.find_angle(im2_yellow, im2_blue)
        joint3 = self.find_angle(im1_yellow, im1_blue)
        # joint3_adj_factor = np.abs(joint2) / 1.57
        # joint3 = (1 - joint3_adj_factor) * joint3 + joint3_adj_factor * self.adjustment(im2_yellow, im2_blue, 3.2)
        self.join2_pub.publish(Float64(joint2))
        self.join3_pub.publish(Float64(joint3))

        print(self.find_distance(im1_yellow, im1_blue))
        print(self.find_distance(im1_center, im1_yellow))

        cv2.imwrite('im1.png', self.cv_image1)
        cv2.imwrite('im2.png', self.cv_image2)
        print('joint2={0} joint3={1}'.format(joint2, joint3))
        # print(np.sqrt(np.sum((im1_yellow - im1_blue) ** 2)))
        # print('adjust_cv1={0}'.format(self.adjustment(im1_yellow, im1_blue, 3.2)))

    def find_color_location_new(self, cv_image, last_pos):
        green = self.detect_color_pos(cv_image, self.green, last_pos[0])
        yellow = self.detect_color_pos(cv_image, self.yellow, last_pos[1])
        blue = self.detect_color_pos(cv_image, self.blue, last_pos[2])
        red = self.detect_color_pos(cv_image, self.red, last_pos[3])
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
        pos_in_cam1 = im1_center - im1_pos
        pos_in_cam2 = im2_center - im2_pos
        x = -pos_in_cam2[0]
        y = -pos_in_cam1[0]
        z = (pos_in_cam2[1] + pos_in_cam1[1]) / 2
        # print('im1_center={0} im1_pos={1}'.format(im1_center, im1_pos))
        # print('im2_center={0} im2_pos={1}'.format(im2_center, im2_pos))
        # print('pos_in_cam1={0}'.format(pos_in_cam1))
        # print('pos_in_cam2={0}'.format(pos_in_cam2))
        return np.array([x, y, z])

    def find_angle_between_2_vector(self,vector1, vector2):
        dot_product = np.dot(vector1, vector2)
        length_vector1 = np.sqrt(np.dot(vector1, vector1))
        length_vector2 = np.sqrt(np.dot(vector2, vector2))
        return np.arccos(dot_product / (length_vector1 * length_vector2))


    def calculate_angle_to_x_y_plane(self, vector):
        [x, y, z] = vector
        xz_plane_projection = [x, 0, z]
        # if z >= 0:
        #     z_vector = [0, 0, 1]
        # else:
        #     z_vector = [0, 0, -1]
        # print('vector={0}'.format(vector))
        # x_angle = np.arcsin(x / np.sqrt(x ** 2 + y ** 2 + z ** 2))
        # y_angle = np.arcsin(y / np.sqrt(x ** 2 + y ** 2 + z ** 2))
        # print('x_angle={0} y_angle={1}'.format(x_angle, y_angle))
        # x_angle = self.find_angle_between_2_vector([vector], xz_plane_projection)
        joint2 = self.find_angle_between_2_vector([0, 0, 1], xz_plane_projection)
        if x < 0:
            joint2 = -joint2
        joint3_projection_vector = [np.sin(joint2),0, np.cos(joint2)]
        # print('joint3_upright_vector={0}'.format(joint3_related_vector))
        joint3 = -self.find_angle_between_2_vector(joint3_projection_vector, vector)
        if y < 0:
            joint3 = -joint3
        return joint3, joint2

    def find_unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

    def find_angle_between_2_vector_signed(self, vector1, vector2):
        dot_product = np.dot(vector1, vector2)
        length_vector1 = np.sqrt(np.dot(vector1, vector1))
        length_vector2 = np.sqrt(np.dot(vector2, vector2))
        return np.arcsin(dot_product / (length_vector1 * length_vector2))

    def calculate_angle_to_previous_joint(self, previous_joint_vector, current_joint_vector, x_angle, y_angle):
        previous_to_current_vector = current_joint_vector - previous_joint_vector
        joint4 = self.find_angle_between_2_vector(previous_joint_vector, previous_to_current_vector)
        # joint4_positive_vector = np.array([1, 0, 0])
        # joint4_positive_vector = np.dot(self.rotation_matrix_y(-y_angle), joint4_positive_vector)
        # joint4_positive_vector = np.dot(self.rotation_matrix_x(x_angle), joint4_positive_vector)
        # print(joint4_positive_vector)
        # print(self.find_angle_between_2_vector(joint4_positive_vector, previous_to_current_vector))
        # if self.find_angle_between_2_vector(joint4_positive_vector, previous_to_current_vector) >= (np.pi /2):
        #     joint4 = -joint4
        return joint4

    def rotation_matrix_y(self, angle):
        return np.array([[np.cos(angle), 0, np.sin(angle)],
                         [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]])

    def rotation_matrix_x(self, angle):
        return np.array([[1, 0, 0],
                         [0, np.cos(angle), -np.sin(angle)],
                         [0, np.sin(angle), np.cos(angle)]])

    def test_function(self):
        im1_center, im1_yellow, im1_blue, im1_red = self.find_color_location_new(self.cv_image1, self.last_pos_1)
        self.last_pos_1 = [im1_center, im1_yellow, im1_blue, im1_red]
        im2_center, im2_yellow, im2_blue, im2_red = self.find_color_location_new(self.cv_image2, self.last_pos_2)
        self.last_pos_2 = [im2_center, im2_yellow, im2_blue, im2_red]
        yellow_blue_vector = self.calculate_3d_pos(im1_yellow, im2_yellow, im1_blue, im2_blue)
        joint3, joint2 = self.calculate_angle_to_x_y_plane(yellow_blue_vector)

        yellow_red_vector = self.calculate_3d_pos(im1_yellow, im2_yellow, im1_red, im2_red)

        joint4 = self.calculate_angle_to_previous_joint(yellow_blue_vector, yellow_red_vector, joint3, joint2)
        # print('test={0}'.format(self.find_angle_between_2_vector_signed(yellow_red_vector, yellow_blue_vector)))


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
        cv2.imwrite('im1.png', self.cv_image1)
        cv2.imwrite('im2.png', self.cv_image2)


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
