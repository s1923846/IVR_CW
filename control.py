#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image


class forward_kinematics:
    def __init__(self):
        # Defines publisher and subscriber
        # initialize the node named
        rospy.init_node('forward_kinematics', anonymous=True)

        self.joint1_angle = 0.0
        self.joint3_angle = 0.0
        self.joint4_angle = 0.0
        self.target_pos = np.array([0, 0, 0])
        self.error_i = 0
        self.flip = False

        # Synchronize subscriptions into one callback
        self.joint1_sub = rospy.Subscriber("joint_angle_1", Float64, self.callback1)
        self.joint3_sub = rospy.Subscriber("joint_angle_3", Float64, self.callback2)
        self.joint4_sub = rospy.Subscriber("joint_angle_4", Float64, self.callback3)
        self.target_sub = rospy.Subscriber("target_pos", Float64MultiArray, self.callback4)
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.curr_end_pos = rospy.Publisher("curr_end_pos", Float64MultiArray, queue_size=10)



        # record the begining time
        self.time = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')

        # initialize error and derivative of error for target tracking
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')

    def callback1(self, data):
        self.joint1_angle = data

    def callback2(self, data):
        self.joint3_angle = data

    def callback3(self, data):
        self.joint4_angle = data

    def callback4(self, data):
        self.target_pos = data

        x = Float64()
        y = Float64()
        z = Float64()

        x, y, z = self.calculate_end_pos(self.joint1_angle, self.joint3_angle, self.joint4_angle)
        print("x: " + str(x))
        print("y: " + str(y))
        print("z: " + str(z))

        curr_end_pos = Float64MultiArray()
        curr_end_pos.data = np.array([x, y, z])

        joint1 = Float64()
        joint3 = Float64()
        joint4 = Float64()
        joint1.data, joint3.data, joint4.data = self.control_close(self.joint1_angle, self.joint3_angle, self.joint4_angle)
        # Publish the results
        try:
            self.robot_joint1_pub.publish(joint1)
            self.robot_joint3_pub.publish(joint3)
            self.robot_joint4_pub.publish(joint4)
            self.curr_end_pos.publish(curr_end_pos)
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

        jacobian = np.array([[a2*np.cos(q1)*np.sin(q2) - a3*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q3)*np.sin(q2)),
                                                        a2*np.cos(q2)*np.sin(q1) + a3*np.cos(q2)*np.cos(q3)*np.sin(q1),
                                                        a3*(np.cos(q1)*np.cos(q3) - np.sin(q1)*np.sin(q2)*np.sin(q3))],
                             [a3*(np.cos(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2)) + a2*np.sin(q1)*np.sin(q2),
                                                       -a2*np.cos(q1)*np.cos(q2) - a3*np.cos(q1)*np.cos(q2)*np.cos(q3),
                                                        a3*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.sin(q2)*np.sin(q3))],
                             [0, - a2*np.sin(q2) - a3*np.cos(q3)*np.sin(q2), -a3*np.cos(q2)*np.sin(q3)]])
        return jacobian

    def control_close(self, q1, q2, q3):
        # P gain
        K_p = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        # D gain
        K_d = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
        # estimate time step
        cur_time = rospy.get_time()
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time

        x, y, z = self.calculate_end_pos(q1, q2, q3)
        pos = np.array([x, y, z])

        # desired target pos
        xyz_d = self.target_pos.data
        x_d = xyz_d[0]
        y_d = xyz_d[1]
        z_d = xyz_d[2]
        pos_d = np.array([x_d, y_d, z_d])
        # estimate derivative of desired target pos
        self.error_d = ((pos_d - pos) - self.error) / dt
        self.error = pos_d - pos

        # calculating the psudeo inverse of Jacobian
        J_inv = np.linalg.pinv(self.calculate_jacobian(q1, q2, q3))

        # control input (angular velocity of joints)
        x = (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose()))
        dq_d = np.dot(J_inv, x)
        # desired joint angles to follow the target
        q1_d = q1.data + 50*dt*dq_d[0]
        q2_d = q2.data + 50*dt*dq_d[1]
        q3_d = q3.data + 50*dt*dq_d[2]
        if q1_d > 0:
            q1_d = min(q1_d, np.pi)
        else:
            q1_d = max(q1_d, -np.pi)
        if q2_d > 0:
            q2_d = min(q2_d, np.pi/2)
        else:
            q2_d = max(q2_d, -np.pi/2)
        if q3_d > 0:
            q3_d = min(q3_d, np.pi/2)
        else:
            q3_d = max(q3_d, -np.pi/2)

        if q2_d < 0:
            q2_d = -q2_d
            q3_d = -q3_d
            if q1_d > 0:
                q1_d = q1_d - np.pi
            else:
                q1_d = q1_d + np.pi
        return np.array([q1_d, q2_d, q3_d])

# run the code if the node is called
# call the class
if __name__ == '__main__':
    fk = forward_kinematics()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
