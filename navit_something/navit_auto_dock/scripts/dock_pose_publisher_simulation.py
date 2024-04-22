#!/usr/bin/env python3
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

dock_pose_=PoseStamped()
dock_pose_.header.frame_id="odom"
dock_pose_.pose.position.x=0.1578954
dock_pose_.pose.position.y=-2.3977
dock_pose_.pose.orientation.w=0.707
dock_pose_.pose.orientation.z=-0.707



def talker():
    pub = rospy.Publisher('dock_point', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub.publish(dock_pose_)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass