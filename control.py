#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
import message_filters


class forward_kinematics:
    def __init__(self):
        # Defines publisher and subscriber
        # initialize the node named
        rospy.init_node('forward_kinematics', anonymous=True)
        # Synchronize subscriptions into one callback
        self.joint1_sub = rospy.Subscriber("joint_angle_1", Float64, self.callback)
        self.joint3_sub = rospy.Subscriber("joint_angle_3", Float64, self.callback)
        self.joint4_sub = rospy.Subscriber("joint_angle_4", Float64, self.callback)
        timeSync = message_filters.TimeSynchronizer([self.joint1_sub, self.joint3_sub, self.joint4_sub], 10)
        timeSync.registerCallback(self.callback)

        target_pos_pub = rospy.Publisher("target_pos", Float64MultiArray, queue_size=10)

        t0 = rospy.get_time()

        self.joint1_angle = 0

        #[cos(q1) * cos(q3) - sin(q1) * sin(q2) * sin(q3), -cos(q2) * sin(q1), cos(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2), a3 * (cos(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) + a2 * sin(q1) * sin(q2)]
        #[cos(q3) * sin(q1) + cos(q1) * sin(q2) * sin(q3), cos(q1) * cos(q2), sin(q1) * sin(q3) - cos(q1) * cos(q3) * sin(q2), a3 * (sin(q1) * sin(q3) - cos(q1) * cos(q3) * sin(q2)) - a2 * cos(q1) * sin(q2)]
        #[-cos(q2) * sin(q3), sin(q2), cos(q2) * cos(q3), a1 + a2 * cos(q2) + a3 * cos(q2) * cos(q3)]
        #[0, 0, 0, 1]
    def callback(self, data1, data2, data3):
        self.joint1_angle = data1
        self.joint3_angle = data2
        self.joint4_angle = data3

        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)
        image = np.concatenate((self.cv_image1, self.cv_image2), axis=1)
        im = cv2.imshow('camera1 and camera2', image)
        cv2.waitKey(1)
        # print(self.pixel2meter(self.cv_image2, False))
        joint2 = Float64()
        joint3 = Float64()
        joint4 = Float64()
        joint2.data, joint3.data, joint4.data = self.joint_angles()
        # Publish the results
        try:
            # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.joint2_pub.publish(joint2)
            self.joint3_pub.publish(joint3)
            self.joint4_pub.publish(joint4)
        except CvBridgeError as e:
            print(e)

# run the code if the node is called
# call the class
# run the code if the node is called
if __name__ == '__main__':
    try:
        forward_kinematics()
    except rospy.ROSInterruptException:
        pass
