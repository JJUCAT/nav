#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
from navit_msgs.msg import StateFeedback
import numpy as np
import math
import tf2_ros
import threading

class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
    def __init__(self):
        # init internals
        self.inited = False
        self.init_received_pose = Pose()
        self.init_yaw = 0.0
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.mutex = threading.Lock()
        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the ackermann
        try:
            arrayIndex = msg.name.index('diff_car::base_link')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.mutex.acquire(1)
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]          
            if not self.inited:
                self.inited = True
                self.init_received_pose = self.last_received_pose
                orientation=(self.init_received_pose.orientation.x, self.init_received_pose.orientation.y, self.init_received_pose.orientation.z, self.init_received_pose.orientation.w)
                (roll,pitch,self.init_yaw) = euler_from_quaternion(orientation)
            self.last_recieved_stamp = rospy.Time.now()
            self.mutex.release()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        if not self.inited:
            return

        if self.mutex.acquire(1):
            cmd = Odometry()
            cmd.header.stamp = self.last_recieved_stamp
            cmd.header.frame_id = 'odom'
            cmd.child_frame_id = 'base_link'
            cmd.pose.pose.position.x = self.last_received_pose.position.x - self.init_received_pose.position.x
            cmd.pose.pose.position.y = self.last_received_pose.position.y - self.init_received_pose.position.y
            cmd.pose.pose.position.z = 0.0
            orientation=(self.last_received_pose.orientation.x, self.last_received_pose.orientation.y, self.last_received_pose.orientation.z, self.last_received_pose.orientation.w)
            (roll,pitch,theta) = euler_from_quaternion(orientation)
            yaw_new= theta-self.init_yaw
            q = quaternion_from_euler(0, 0, yaw_new)
            cmd.pose.pose.orientation.x= q[0]
            cmd.pose.pose.orientation.y= q[1]
            cmd.pose.pose.orientation.z= q[2]
            cmd.pose.pose.orientation.w= q[3]
            cmd.twist.twist = self.last_received_twist
            self.pub_odom.publish(cmd)
            tf = TransformStamped(
                header=Header(
                    frame_id=cmd.header.frame_id,
                    stamp=cmd.header.stamp
                ),
                child_frame_id=cmd.child_frame_id,
                transform=Transform(
                    translation=cmd.pose.pose.position,
                    rotation=cmd.pose.pose.orientation
                )
            )
            self.tf_pub.sendTransform(tf)
        self.mutex.release()

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
