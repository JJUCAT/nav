#!/usr/bin/env python3
import rospy
import actionlib
from move_base.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from geometry_msgs.msg import PoseStamped

PLANNER_ACTION_NAME = "/move_base"
planner_client = actionlib.SimpleActionClient(PLANNER_ACTION_NAME, MoveBaseAction)
planner_action_goal = MoveBaseGoal()
planner_result = MoveBaseResult()
planner_plugins = "topoint"

def recvRvizPoint(msg):
  rospy.loginfo("recv goal point [%.3f, %.3f]", msg.point.x, msg.point.y)
  planner_action_goal.target_pose.pose.position = msg.point
  planner_client.publish(planner_action_goal)

if __name__ == "__main__":
    rospy.init_node("rrt_test")
    planner_client.wait_for_server()

    rospy.Subscriber("/clicked_point", recvRvizPoint)

    rospy.spin()
