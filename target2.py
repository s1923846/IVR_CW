#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64


def movement1_publisher():
    # Defines publisher and subscriber
    # initialize the node named
    rospy.init_node('movement_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50hz

    robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        cur_time = np.array([rospy.get_time()]) - t0
        # y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
        tx = 3.0 * np.cos(cur_time * np.pi / 20)
        ty = 4.0 * np.sin(cur_time * np.pi / 14) + 0.5
        tz = 1.0 * np.sin(cur_time * np.pi / 18) + 4.5
        target_pos = Float64MultiArray()
        print("x: " + str(tx))
        print("y: " + str(ty))
        print("z: " + str(tz))
        target_pos.data = np.array([tx, ty, tz])

        rate.sleep()


# run the code if the node is called
if __name__ == '__main__':
    try:
        movement1_publisher()
    except rospy.ROSInterruptException:
        pass