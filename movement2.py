#!/usr/bin/env python3
import math

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64


def movement2_publisher():
    # Defines publisher and subscriber
    # initialize the node named
    rospy.init_node('movement_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50hz

    robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        cur_time = np.array([rospy.get_time()]) - t0
        joint1 = Float64()
        joint3 = Float64()
        joint4 = Float64()
        joint1.data = np.pi * np.sin(cur_time * np.pi / 28)
        joint3.data = (np.pi / 2) * np.sin(cur_time * np.pi / 20)
        joint4.data = (np.pi / 2) * np.sin(cur_time * np.pi / 18)

        robot_joint1_pub.publish(joint1)
        robot_joint3_pub.publish(joint3)
        robot_joint4_pub.publish(joint4)

        rate.sleep()


# run the code if the node is called
if __name__ == '__main__':
    try:
        movement2_publisher()
    except rospy.ROSInterruptException:
        pass
