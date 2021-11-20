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

        self.last_red = np.array([0, 0, 0])
        self.last_blue = np.array([0, 0, 0])
        self.cv_image1 = np.array([])
        self.cv_image2 = np.array([])
        # Hardcode the y coordinate of green and yellow sphere because they don't move.
        self.green_center = np.array([399, 400, 543])
        self.yellow_center = np.array([399, 399, 440])
        self.last_sensible_j2 = 0


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


        #self.pixel2meter_1 = 0.0375
        #self.pixel2meter_2 = 0.035086369433113516
        #self.yellow_center_2 = np.array([401, 430])
        # 0.03802281



    # Recieve data from camera 1, process it, and publish
    def callback1(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #print(self.pixel2meter(self.cv_image1, True))
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
        joint2 = Float64()
        joint3 = Float64()
        joint4 = Float64()
        joint2.data, joint3.data, joint4.data = self.joint_angles()
        # Publish the results
        try:
            #self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.joint2_pub.publish(joint2)
            self.joint3_pub.publish(joint3)
            self.joint4_pub.publish(joint4)
        except CvBridgeError as e:
            print(e)
    # In this method you can focus on detecting the centre of the red circle
    def detect_red(self, image, is_image1):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)

        if is_image1:
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_red[1] = cx
                self.last_red[2] = cy
            return np.array(self.last_red)
        else:
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_red[0] = cx
                self.last_red[2] = cy
            return np.array(self.last_red)

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
    def detect_blue(self, image, is_image1):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)

        if is_image1:
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_blue[1] = cx
                self.last_blue[2] = cy
            return np.array(self.last_blue)
        else:
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_blue[0] = cx
                self.last_blue[2] = cy
            return np.array(self.last_blue)

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
        circle1Pos = self.detect_green(image)#, is_image1)
        circle2Pos = self.detect_yellow(image)
        # find the distance between two circles

        #temp2 = self.detect_yellow(image)
        #temp1 = self.detect_blue(image)
        #a = np.linalg.norm(np.array(temp1[0],temp1[2])-temp2)
        #print("length: " + str(a))

        #print('pos1' + str(circle1Pos))
        #print('pos2' + str(circle2Pos))
        dist = circle1Pos[1] - circle2Pos[1]
        return 10 / dist

    def calculate_pos(self, color):
        pos_1 = np.array([])
        pos_2 = np.array([])
        if color == 'red':
            pos_1 = self.detect_red(self.cv_image1, True)
            pos_2 = self.detect_red(self.cv_image2, False)

        elif color == 'blue':
            pos_1 = self.detect_blue(self.cv_image1, True)
            pos_2 = self.detect_blue(self.cv_image2, False)
            above_yellow1 = (pos_1[2] - self.yellow_center[2]) < 0
            above_yellow2 = (pos_2[2] - self.yellow_center[2]) < 0
            if not (above_yellow1 & above_yellow2):
                #print(color)
                #print(np.array([(pos_2[0] - self.green_center[0]), (pos_1[1] - self.green_center[1]), 104]))
                return np.array([(pos_2[0] - self.green_center[0]), (pos_1[1] - self.green_center[1]), 102])
        x = (pos_2[0] - self.green_center[0])
        y = (pos_1[1] - self.green_center[1])
        z_1 = (pos_1[2] - self.green_center[2])
        z_2 = (pos_2[2] - self.green_center[2])
        z = (-z_1 - z_2) / 2
        #print(color)
        #print(np.array([x, y, z]))
        return np.array([x, y, z])

    def all_coordinates(self):
        green = np.array([0, 0, 0])
        yellow = np.array([0, 0, 102])
        blue = self.calculate_pos('blue')
        print("b")
        print(blue)
        red = self.calculate_pos('red')
        return [green, yellow, blue, red]

    def calc_angle(self, v1, v2):
        d = np.dot(v1, v2)
        l1 = np.linalg.norm(v1)
        l2 = np.linalg.norm(v2)
        angle = np.arccos(d / (l1 * l2))
        return angle

    def joint_angles(self):
        green, yellow, blue, red = self.all_coordinates()
        yellow2blue = blue - yellow

        #yellow2blue_xz = np.array([yellow2blue[0], yellow2blue[2]])
        blue2red = red - blue
        #print(yellow2blue_xz)
        #joint2 = self.calc_angle(yellow2blue, yellow)

        joint3 = self.calc_angle(yellow2blue, np.array([0, 1, 0]))
        joint3 = - np.pi / 2 + joint3
        if joint3 > np.pi / 2:
            joint3 = np.pi - joint3
        if joint3 < (- np.pi / 2):
            joint3 = - np.pi + joint3

        transformed_x = np.cross(yellow2blue, np.array([0, 1, 0]))
        print("XXXXXXXXXXXXXXXX" + str(transformed_x))
        print("yellow to blue: " + str(yellow2blue))
        joint2 = self.calc_angle(transformed_x, np.array([1, 0, 0]))
        print("joint2: " + str(joint2))
        if joint2 > np.pi / 2:
            joint2 = np.pi - joint2
        if joint2 < (- np.pi / 2):
            joint2 = - np.pi + joint2

        if np.absolute(joint3 - np.pi) < 0.15:
            joint2 = self.last_sensible_j2 + (np.pi / 2) * np.sin(np.pi / 15)

        self.last_sensible_j2 = joint2



        joint4 = self.calc_angle(yellow2blue, blue2red)
        if joint4 > np.pi / 2:
            joint4 = np.pi - joint4
        if joint4 < (- np.pi / 2):
            joint4 = np.pi + joint4

        print(joint2)
        print(joint3)
        return [joint2, joint3, joint4]



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


