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
import message_filters


class image_converter:
    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=10)
        # initialize a publisher to send joint angle to a topic named joint_angle_2
        self.joint2_pub = rospy.Publisher("joint_angle_2", Float64, queue_size=10)
        # initialize a publisher to send joint angle to a topic named joint_angle_2
        self.joint3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        # initialize a publisher to send joint angle to a topic named joint_angle_2
        self.joint4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)

        #sub = message_filters.Subscriber("pose_topic", robot_msgs.msg.Pose)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)

        # initialize a subscriber to recieve messages rom a topic named /robot/camera2/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        #timesync = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
        #timesync.registerCallback(self.callback1)
        #timesync.registerCallback(self.callback2)

        self.cv_image1 = np.array([])
        self.cv_image2 = np.array([])

        # Hardcode the y coordinate of green and yellow sphere because they don't move.
        self.green_center = np.array([395, 405, 542])
        self.yellow_center = np.array([401, 402, 430])
        self.pixel2meter_1 = 0.0375
        self.pixel2meter_2 = 0.035086369433113516
        #self.yellow_center_2 = np.array([401, 430])
# 0.03802281



    # Recieve data from camera 1, process it, and publish
    def callback1(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        print(self.pixel2meter(self.cv_image1, True))
        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)



    # Recieve data from camera 2, process it, and publish
    def callback2(self, data):
        # Recieve the image
        try:
            #self.cv_image1 = self.bridge.imgmsg_to_cv2(data_1, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)
        image = np.concatenate((self.cv_image1, self.cv_image2), axis=1)
        im = cv2.imshow('camera1 and camera2', image)
        cv2.waitKey(1)
        #print(self.pixel2meter(self.cv_image2, False))
        self.joint_angle_2 = Float64()
        self.joint_angle_2.data = self.calculate_joint_angle()
        self.joint_angle_3 = Float64()
        self.joint_angle_3.data = self.calculate_joint_angle()
        self.joint_angle_4 = Float64()
        self.joint_angle_4.data = self.calculate_joint_angle()

        # Publish the results
        try:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.joint2_pub.publish(self.joint_angle_2)
            self.joint3_pub.publish(self.joint_angle_3)
            self.joint4_pub.publish(self.joint_angle_4)


        except CvBridgeError as e:
            print(e)
    # In this method you can focus on detecting the centre of the red circle
    def detect_red(self, image, is_image1):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)

        # If red is blocked by blue
        if M['m00'] == 0:
            # we can use y coordinate of the blue as that of the red
            if is_image1:
                return np.array([0, self.detect_blue(image, True)[1], 0])
            else:
                # we can use x coordinate of the blue as that of the red
                return np.array([self.detect_blue(image, False)[0], 0, 0])
        else:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if is_image1:
                return np.array([0, cx, cy])
            else:
                return np.array([cx, 0, cy])

    # Detecting the centre of the blue circle
    def detect_blue(self, image,is_image1):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)

        # If the blue is blocked by the yellow
        if M['m00'] == 0:
            # we can use y coordinate of the yellow as that of the blue
            if is_image1:
                return np.array([0, self.yellow_center[1], 0])
            else:
                # we can use x coordinate of the yellow as that of the blue
                return np.array([self.yellow_center[0], 0, 0])
        else:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if is_image1:
                return np.array([0, cx, cy])
            else:
                return np.array([cx, 0, cy])

    # Detecting the centre of the yellow circle
    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    # Detecting the centre of the green circle
    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    # Calculate the conversion from pixel to meter
    def pixel2meter(self, image,is_image1):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_red(image, is_image1)
        circle2Pos = self.detect_yellow(image)
        # find the distance between two circles

        #temp2 = self.detect_yellow(image)
        #temp1 = self.detect_blue(image)
        #a = np.linalg.norm(np.array(temp1[0],temp1[2])-temp2)
        #print("length: " + str(a))

        print('pos1' + str(circle1Pos))
        print('pos2' + str(circle2Pos))
        dist = circle1Pos[2] - circle2Pos[1]
        return 10 / dist

    # Calculate the relevant joint angles from the image
    def detect_joint_angles(self, image):
        a = self.pixel2meter(image)
        # Obtain the centre of each coloured blob
        center = a * self.detect_yellow(image)
        circle1Pos = a * self.detect_blue(image)
        circle2Pos = a * self.detect_green(image)
        circle3Pos = a * self.detect_red(image)
        # Solve using trigonometry
        ja1 = np.arctan2(center[0] - circle1Pos[0], center[1] - circle1Pos[1])
        ja2 = np.arctan2(circle1Pos[0] - circle2Pos[0], circle1Pos[1] - circle2Pos[1]) - ja1
        ja3 = np.arctan2(circle2Pos[0] - circle3Pos[0], circle2Pos[1] - circle3Pos[1]) - ja2 - ja1
        return np.array([ja1, ja2, ja3])

    def calculate_pos(self, color):
        pos_1 = np.array([])
        pos_2 = np.array([])
        if(color == 'red'):
            pos_1 = self.detect_red(self.cv_image1)
            pos_2 = self.detect_red(self.cv_image2)
        elif(color == 'blue'):
            pos_1 = self.detect_blue(self.cv_image1)
            pos_2 = self.detect_blue(self.cv_image2)
        x = (pos_2[0] - self.green_center[0]) * self.pixel2meter_2
        y = (pos_1[1] - self.green_center[1]) * self.pixel2meter_1
        z_1 = (pos_1[2] - self.green_center[2]) * self.pixel2meter_1
        z_2 = (pos_2[2] - self.green_center[2]) * self.pixel2meter_2
        z = (z_1 + z_2) / 2
        return np.array([x, y, z])






    def calculate_joint_angle(self):
        return 1.0;
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


