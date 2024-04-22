#!/usr/bin/env python3
import math
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped

dock_pose_=PoseStamped()
dock_pose_.header.frame_id="odom"
dock_pose_.pose.position.x=2.5
dock_pose_.pose.position.y=0
dock_pose_.pose.orientation.w=1.0
dock_pose_.pose.orientation.z=0

def talker():
    pub = rospy.Publisher('dock_point',PoseStamped,queue_size=10)
    rospy.init_node('talker',anonymous=True)
    pub.publish(dock_pose_)
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
