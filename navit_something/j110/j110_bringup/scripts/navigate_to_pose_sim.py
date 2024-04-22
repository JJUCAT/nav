#!/usr/bin/env python3
import sys
import rospy
import actionlib
import tf
import math
import copy
import numpy as np
from navit_msgs.msg import NavigateToPoseAction, NavigateToPoseGoal, NavigateToPoseFeedback
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

client = actionlib.SimpleActionClient("/navigate_to_pose", NavigateToPoseAction)
goal = NavigateToPoseGoal()

def NavigateToPoseCallback(state, result):
    rospy.loginfo("to pose task is Done")

def NavigateToPoseFeedbackCallback(fb):
    rospy.loginfo("to pose task feedback ...")
    rospy.loginfo("distance to goal %f" %(fb.distance_to_goal))
    rospy.loginfo("completion percentage %f" %(fb.completion_percentage))

def recvGoal(p):
    rospy.loginfo("recv goal [%.3f, %.3f]", p.pose.position.x, p.pose.position.y)
    goal.start = PoseStamped()
    goal.goal = p
    goal.use_start = False
    goal.arrival_mode = 0
    goal.xy_goal_tolerance = 0.1
    goal.yaw_goal_tolerance = 0.17
    rospy.loginfo("send <navigate to pose> goal ...")
    client.send_goal(goal, done_cb=NavigateToPoseCallback, active_cb=None, feedback_cb=NavigateToPoseFeedbackCallback)

if __name__ == "__main__":
    rospy.init_node("navigate_to_pose_sim_node")
    rospy.loginfo("navigate to pose sim node start.")
    rospy.loginfo("Connecting to server </navigate_to_pose> ...")
    client.wait_for_server()
    rospy.loginfo("Connected.")
    goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, recvGoal, queue_size=1)
    rospy.spin()
