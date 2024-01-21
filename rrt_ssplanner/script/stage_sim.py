#! /usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal, MoveBaseResult
from geometry_msgs.msg import PoseStamped, PointStamped

PLANNER_ACTION_NAME = "/move_base"
planner_client = actionlib.SimpleActionClient(PLANNER_ACTION_NAME, MoveBaseAction)
planner_action_goal = MoveBaseGoal()
planner_result = MoveBaseResult()
planner_plugins = "topoint"

def active_callback():
  rospy.loginfo("active ...")

def feedback_callback():
  rospy.loginfo("feedback ...")

def result_callback(state, result):
    rospy.loginfo("result ...")

def recvRvizPoint(msg):
  rospy.loginfo("recv goal point [%.3f, %.3f]", msg.point.x, msg.point.y)
  point_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
  goal = MoveBaseActionGoal()
  goal.goal.target_pose.header.frame_id = "map"
  goal.goal.target_pose.pose.orientation.w = 1
  goal.goal.target_pose.pose.position = msg.point
  point_pub.publish(goal)

if __name__ == "__main__":
  rospy.init_node("rrt_test")
  rospy.loginfo("rrt sim start.")
  planner_client.wait_for_server()
  rospy.Subscriber("/clicked_point", PointStamped, recvRvizPoint, queue_size=1)
  rospy.spin()
