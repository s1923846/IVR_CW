#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
import message_filters
from sensor_msgs.msg import Image


class forward_kinematics:
    def __init__(self):
        # Defines publisher and subscriber
        # initialize the node named
        rospy.init_node('forward_kinematics', anonymous=True)

        # Synchronize subscriptions into one callback
        self.joint1_sub = message_filters.Subscriber("joint_angle_1", Float64)
        self.joint3_sub = message_filters.Subscriber("joint_angle_3", Float64)
        self.joint4_sub = message_filters.Subscriber("joint_angle_4", Float64)
        self.target_sub = message_filters.Subscriber("target_pos", Float64MultiArray)
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        timeSync = message_filters.ApproximateTimeSynchronizer([self.joint1_sub, self.joint3_sub,
                                                                self.joint4_sub, self.target_sub],
                                                               10, 0.1, allow_headerless=True)
        timeSync.registerCallback(self.callback)

        self.joint1_angle = 0
        self.joint3_angle = 0
        self.joint4_angle = 0
        self.target_pos = np.array([0, 0, 0])

        # record the begining time
        self.time = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
        # initialize error and derivative of error for target tracking
        self.error = np.array([0.0, 0., 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')

        #[cos(q1) * cos(q3) - sin(q1) * sin(q2) * sin(q3), -cos(q2) * sin(q1), cos(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2), a3 * (cos(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) + a2 * sin(q1) * sin(q2)]
        #[cos(q3) * sin(q1) + cos(q1) * sin(q2) * sin(q3), cos(q1) * cos(q2) , sin(q1) * sin(q3) - cos(q1) * cos(q3) * sin(q2), a3 * (sin(q1) * sin(q3) - cos(q1) * cos(q3) * sin(q2)) - a2 * cos(q1) * sin(q2)]
        #[-cos(q2) * sin(q3)                             , sin(q2)           , cos(q2) * cos(q3)                              , a1 + a2 * cos(q2) + a3 * cos(q2) * cos(q3)]
        #[0                                              , 0                 , 0                                              , 1]

       # Jacobian matrix

        # [0, sin(q1) * sin(q2), cos(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2),
        # a2 * cos(q1) * sin(q2) - a3 * (sin(q1) * sin(q3) - cos(q1) * cos(q3) * sin(q2)),
        # a2 * cos(q2) * sin(q1) + a3 * cos(q2) * cos(q3) * sin(q1),
        # a3 * (cos(q1) * cos(q3) - sin(q1) * sin(q2) * sin(q3))]

        # [0, -cos(q1) * sin(q2), sin(q1) * sin(q3) - cos(q1) * cos(q3) * sin(q2),
        # a3 * (cos(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) + a2 * sin(q1) * sin(q2),
        # - a2 * cos(q1) * cos(q2) - a3 * cos(q1) * cos(q2) * cos(q3),
        # a3 * (cos(q3) * sin(q1) + cos(q1) * sin(q2) * sin(q3))]

        # [1, cos(q2), cos(q2) * cos(q3), 0, - a2 * sin(q2) - a3 * cos(q3) * sin(q2), -a3 * cos(q2) * sin(q3)]

    def callback(self, data1, data2, data3, data4):
        self.joint1_angle = data1
        self.joint3_angle = data2
        self.joint4_angle = data3
        self.target_pos = data4

        x = Float64()
        y = Float64()
        z = Float64()

        x, y, z = self.calculate_end_pos(self.joint1_angle, self.joint3_angle, self.joint4_angle)
        print("x: " + str(x))
        print("y: " + str(y))
        print("z: " + str(z))

        joint1 = Float64()
        joint3 = Float64()
        joint4 = Float64()
        joint1.data, joint3.data, joint4.data = self.control_open(self.joint1_angle, self.joint3_angle, self.joint4_angle)
        # Publish the results
        try:
            self.robot_joint1_pub.publish(joint1)
            self.robot_joint3_pub.publish(joint3)
            self.robot_joint4_pub.publish(joint4)
        except rospy.ROSInterruptException as e:
            print(e)

    def calculate_end_pos(self, q1, q2, q3):
        a1 = 4
        a2 = 3.2
        a3 = 2.8
        q1 = q1.data
        q2 = q2.data
        q3 = q3.data
        x = a3 * (np.cos(q1) * np.sin(q3) + np.cos(q3) * np.sin(q1) * np.sin(q2)) + a2 * np.sin(q1) * np.sin(q2)
        y = a3 * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q3) * np.sin(q2)) - a2 * np.cos(q1) * np.sin(q2)
        z = a1 + a2 * np.cos(q2) + a3 * np.cos(q2) * np.cos(q3)
        return x, y, z

    def calculate_blue_pos(self, q1, q2):
        a1 = 4
        a2 = 3.2
        q1 = q1.data
        q2 = q2.data
        x = a2 * np.sin(q1) * np.sin(q2)
        y = -a2 * np.cos(q1) * np.sin(q2)
        z = a1 + a2 * np.cos(q2)
        return x, y, z

    # Calculate the robot Jacobian
    def calculate_jacobian(self, q1, q2, q3):
        a2 = 3.2
        a3 = 2.8
        q1 = q1.data
        q2 = q2.data
        q3 = q3.data

        jacobian = np.array([[0, np.sin(q1)*np.sin(q2), np.cos(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2),
                    a2*np.cos(q1)*np.sin(q2) - a3*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q3)*np.sin(q2)),
                    a2 * np.cos(q2) * np.sin(q1) + a3 * np.cos(q2) * np.cos(q3) * np.sin(q1),
                    a3 * (np.cos(q1) * np.cos(q3) - np.sin(q1) * np.sin(q2) * np.sin(q3))],
                    [0, -np.cos(q1) * np.sin(q2), np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q3) * np.sin(q2),
                    a3 * (np.cos(q1) * np.sin(q3) + np.cos(q3) * np.sin(q1) * np.sin(q2)) + a2 * np.sin(q1) * np.sin(q2),
                    - a2 * np.cos(q1) * np.cos(q2) - a3 * np.cos(q1) * np.cos(q2) * np.cos(q3),
                    a3 * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.sin(q2) * np.sin(q3))],
                    [1, np.cos(q2), np.cos(q2) * np.cos(q3), 0, - a2 * np.sin(q2) - a3 * np.cos(q3) * np.sin(q2),
                     -a3 * np.cos(q2) * np.sin(q3)]])
        return jacobian

        # Estimate control inputs for open-loop control

    def control_open(self, q1, q2, q3):
        # estimate time step
        cur_time = rospy.get_time()
        dt = cur_time - self.time_previous_step2
        self.time_previous_step2 = cur_time
        # calculating the psudeo inverse of Jacobian
        J_inv = np.linalg.pinv(self.calculate_jacobian(q1, q2, q3))
        x, y, z = self.calculate_end_pos(q1, q2, q3)
        pos = np.array([x, y, z])
        # desired target pos
        pos_d = self.target_pos.data
        # estimate derivative of desired target pos
        self.error = (pos_d - pos) / dt
        # desired joint angles to follow the target
        q1_d = q1.data + (dt * np.dot(J_inv, self.error.transpose()))[0]
        q2_d = q2.data + (dt * np.dot(J_inv, self.error.transpose()))[1]
        q3_d = q3.data + (dt * np.dot(J_inv, self.error.transpose()))[2]
        return np.array([q1_d, q2_d, q3_d])

# run the code if the node is called
# call the class
if __name__ == '__main__':
    fk = forward_kinematics()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

