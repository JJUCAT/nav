#!/usr/bin/python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


rospy.init_node("test_ackermann_vel_smoother")

test_pub = rospy.Publisher("raw_cmd_vel_test_ackermann",AckermannDriveStamped, queue_size =1)

test_cmd = AckermannDriveStamped()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    t = rospy.Time.now()
    cmd_speed = 1.5 * np.sin(2*t.to_sec())
    cmd_steer = 0.8 * np.sin(0.5 *t.to_sec())
    rospy.loginfo("vel cmd is {}".format(cmd_speed))
    test_cmd.header.stamp = rospy.Time.now()
    test_cmd.drive.speed = cmd_speed
    test_cmd.drive.steering_angle = cmd_steer
    test_pub.publish(test_cmd)

    r.sleep()
