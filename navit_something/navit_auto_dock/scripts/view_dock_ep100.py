#!/usr/bin/env python3

import copy
from math import cos, sin

import rospy
import tf2_ros
from navit_msgs.msg import ApproachDockActionFeedback, FinalDockActionFeedback
from geometry_msgs.msg import PoseStamped, TransformStamped

class ViewDock(object):

    def __init__(self, publish_tf=True):
        rospy.init_node("view_dock")
        rospy.Subscriber("approach_dock/feedback", ApproachDockActionFeedback, self.callback)
        rospy.Subscriber("final_dock/feedback", FinalDockActionFeedback, self.final_dock_callback)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.publish_tf = publish_tf
    
    def final_dock_callback(self, msg):
        if self.publish_tf:
            pose = msg.feedback.dock_pose_feedback.pose
            t = TransformStamped()
            t.header.stamp = rospy.Time.now() 
            t.header.frame_id = msg.feedback.dock_pose_feedback.header.frame_id
            t.child_frame_id = "final_dock_pose"
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w
            self.broadcaster.sendTransform(t)
            

    def callback(self, msg):
        # Publish TF for dock
        if self.publish_tf:
            pose = msg.feedback.dock_pose.pose
            t = TransformStamped()
            t.header.stamp = rospy.Time.now() 
            t.header.frame_id = msg.feedback.dock_pose.header.frame_id
            t.child_frame_id = "percepted_dock"
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w

            self.broadcaster.sendTransform(t)

            _pose = msg.feedback.target_pose.pose
            _t = TransformStamped()
            _t.header.stamp = rospy.Time.now() 
            _t.header.frame_id = msg.feedback.target_pose.header.frame_id
            _t.child_frame_id = "pre_charging_pose"
            _t.transform.translation.x = _pose.position.x
            _t.transform.translation.y = _pose.position.y
            _t.transform.translation.z = _pose.position.z
            _t.transform.rotation.x = _pose.orientation.x
            _t.transform.rotation.y = _pose.orientation.y
            _t.transform.rotation.z = _pose.orientation.z
            _t.transform.rotation.w = _pose.orientation.w

            self.broadcaster.sendTransform(_t)

if __name__ == "__main__":
    v = ViewDock()
    rospy.spin()
