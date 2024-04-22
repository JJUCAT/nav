#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped


dock_pose_=PoseStamped()
dock_pose_.header.frame_id="base_link"
dock_pose_.pose.position.x=-1.5
dock_pose_.pose.position.y=0
dock_pose_.pose.orientation.w=0
dock_pose_.pose.orientation.z=1

def talker():
    pub = rospy.Publisher('dock_point', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub.publish(dock_pose_)
    


if __name__ == '__main__':
    rospy.sleep(5.0)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

