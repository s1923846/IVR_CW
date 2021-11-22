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


class image_converter:
    # Defines publisher and subscriber
    def __init__(self):
        self.last_red1 = np.array([0, 0])
        self.last_red2 = np.array([0, 0])
        self.last_blue1 = np.array([0, 0])
        self.last_blue2 = np.array([0, 0])
        self.green_center_1 = np.array([400, 544])
        self.green_center_2 = np.array([399, 540])
        self.yellow_center_1 = np.array([400, 440])
        self.yellow_center_2 = np.array([400, 440])
        self.last_joint1 = 0
        self.j3_close_to_0 = True
        self.j3_positive = True
        self.last_joint3 = 0
        self.cv_image1 = np.array([])
        self.cv_image2 = np.array([])

        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1

        self.joint1_pub = rospy.Publisher("joint_angle_1", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera2/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # Hardcode the y coordinate of green and yellow sphere because they don't move.

    def callback1(self, data_1):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data_1, "bgr8")
        except CvBridgeError as e:
            print(e)

    # Recieve data from camera 2, process it, and publish
    def callback2(self, data_2):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data_2, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)

        im1 = cv2.imshow('window1', self.cv_image2)
        cv2.waitKey(1)
        # Publish the results
        try:
            joint1 = Float64()
            joint3 = Float64()
            joint4 = Float64()
            joint1.data, joint3.data, joint4.data = self.joint_angle_1()

            self.joint1_pub.publish(joint1)
            self.joint3_pub.publish(joint3)
            self.joint4_pub.publish(joint4)
        except CvBridgeError as e:
            print(e)

    # In this method you can focus on detecting the centre of the red circle.
    def detect_red(self, image, is_image1):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if is_image1:
            if M['m00'] != 0:
                self.last_red1 = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])]
            return np.array(self.last_red1)
        else:
            if M['m00'] != 0:
                self.last_red2 = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])]
            return np.array(self.last_red2)

    # Detecting the centre of the blue circle
    def detect_blue(self, image, is_image1):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if is_image1:
            if M['m00'] != 0:
                self.last_blue1 = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])]
            return np.array(self.last_blue1)
        else:
            if M['m00'] != 0:
                self.last_blue2 = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])]
            return np.array(self.last_blue2)

    # Calculate the conversion from pixel to meter
    def pixel2meter(self, image, is_image1):
        # Obtain the centre of each coloured blob
        circle1Pos = np.array([])
        circle2Pos = np.array([])
        if is_image1:
            circle1Pos = self.yellow_center_1
            circle2Pos = self.green_center_1
        else:
            circle1Pos = self.yellow_center_2
            circle2Pos = self.green_center_2
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 4 / np.sqrt(dist)

    def x_coordinate(self, color):
        # a = self.pixel2meter(self.cv_image1, True)
        vector = np.array([])
        if color == "red":
            vector = self.detect_red(self.cv_image2, False) - self.green_center_2
        elif color == "blue":
            vector = self.detect_blue(self.cv_image2, False) - self.green_center_2
        return vector[0]

    def y_coordinate(self, color):
        # a = self.pixel2meter(self.cv_image1, True)
        vector = np.array([])
        if color == "red":
            vector = self.detect_red(self.cv_image1, True) - self.green_center_1
        elif color == "blue":
            vector = self.detect_blue(self.cv_image1, True) - self.green_center_1
        return vector[0]

    def z_coordinate(self, color):
        # a1 = self.pixel2meter(self.cv_image1, True)
        # a2 = self.pixel2meter(self.cv_image2, False)
        vector1 = np.array([])
        vector2 = np.array([])
        if color == "red":
            vector1 = self.detect_red(self.cv_image1, True) - self.green_center_1
            vector2 = self.detect_red(self.cv_image2, False) - self.green_center_2
        elif color == "blue":
            above_yellow1 = (self.detect_blue(self.cv_image1, True)[1] - self.yellow_center_1[1]) < 0
            above_yellow2 = (self.detect_blue(self.cv_image2, False)[1] - self.yellow_center_2[1]) < 0
            if not (above_yellow1 & above_yellow2):
                return 102
            vector1 = self.detect_blue(self.cv_image1, True) - self.green_center_1
            vector2 = self.detect_blue(self.cv_image2, False) - self.green_center_2
        return - (vector1[1] + vector2[1]) / 2

    def all_coordinates(self):
        green = np.array([0, 0, 0])
        yellow = np.array([0, 0, 102])
        x_blue = self.x_coordinate("blue")
        y_blue = self.y_coordinate("blue")
        z_blue = self.z_coordinate("blue")
        blue = np.array([x_blue, y_blue, z_blue])
        x_red = self.x_coordinate("red")
        y_red = self.y_coordinate("red")
        z_red = self.z_coordinate("red")
        red = np.array([x_red, y_red, z_red])
        return [green, yellow, blue, red]

    def calc_angle(self, v1, v2):
        d = np.dot(v1, v2)
        l1 = np.linalg.norm(v1)
        l2 = np.linalg.norm(v2)
        angle = np.arccos(d / (l1 * l2))
        return angle

    def joint_angle_1(self):
        green, yellow, blue, red = self.all_coordinates()
        print('x: ' + str(red[0] * 0.038))
        print('y: ' + str(red[1] * 0.038))
        print('z: ' + str(red[2] * 0.038))
        yellow2blue = blue - yellow
        yellow2blue_xy = np.array([yellow2blue[0], yellow2blue[1]])
        blue2red = red - blue

        # assuming joint3 is always positive
        joint3 = np.absolute(self.calc_angle(yellow2blue, np.array(yellow)))
        if joint3 > np.pi / 2:
            joint3 = np.pi - joint3

        # if joint3 is always positive, there exit case that joint1 suddenly turn 180 degree
        # but such error does not affect inverse kinematics
        joint1 = self.calc_angle(yellow2blue_xy, np.array([0, -1]))
        if yellow2blue_xy[0] < 0:
            joint1 = -joint1

        y_after_joint1 = np.array([-np.sin(joint1), np.cos(joint1), 0])
        x_after_joint1 = np.array([np.cos(joint1), np.sin(joint1), 0])

        joint4 = np.absolute(self.calc_angle(yellow2blue, blue2red))
        if joint4 > np.pi / 2:
            joint4 = np.pi - joint4
        if np.dot(blue2red, x_after_joint1) < 0:
            joint4 = -joint4

        return [joint1, joint3, joint4]

    def joint_angles_2(self):
        green, yellow, blue, red = self.all_coordinates()
        yellow2blue = blue - yellow
        yellow2blue_xy = np.array([yellow2blue[0], yellow2blue[1]])
        blue2red = red - blue

        joint3 = self.calc_angle(yellow2blue, np.array(yellow))
        if joint3 > np.pi / 2:
            joint3 = np.pi - joint3
        if joint3 < (- np.pi / 2):
            joint3 = - np.pi + joint3

        joint1 = self.calc_angle(yellow2blue_xy, np.array([0, -1]))
        if yellow2blue_xy[0] < 0:
            joint1 = -joint1

        diff = np.absolute(joint1 - self.last_joint1)
        #print(diff)
        if diff > 5:
            joint1 = -joint1
        diff = np.absolute(joint1 - self.last_joint1)
        if np.absolute(joint1 - np.pi - self.last_joint1) < diff:
            joint1 = joint1 - np.pi
        elif np.absolute(joint1 + np.pi - self.last_joint1) < diff:
            joint1 = joint1 + np.pi

        if np.absolute(joint3) < 0.15:
            joint1 = self.last_joint1

        if joint1 > np.pi:
            joint1 = np.pi
        if joint1 < -np.pi:
            joint1 = -np.pi

        self.last_joint1 = joint1

        y_after_joint1 = np.array([-np.sin(joint1), np.cos(joint1), 0])
        x_after_joint1 = np.array([np.cos(joint1), np.sin(joint1), 0])
        if np.dot(yellow2blue, y_after_joint1) > 0:
            joint3 = -joint3

        joint4 = self.calc_angle(yellow2blue, blue2red)
        if joint4 > np.pi / 2:
            joint4 = np.pi - joint4
        if joint4 < (- np.pi / 2):
            joint4 = np.pi + joint4
        if np.dot(blue2red, x_after_joint1) < 0:
            joint4 = -joint4
        return [joint1, joint3, joint4]


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
