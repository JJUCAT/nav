#!/usr/bin/env python
import rospy
import actionlib
from navit_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

import numpy as np

ACTION_NAME = "/navit_move_base" 
client = actionlib.SimpleActionClient(ACTION_NAME, MoveBaseAction)
action_goal = MoveBaseGoal()

def getRandomGoal(action_goal):
    rand_planner = np.random.random_integers(0,2)
    [x,y] = np.random.randn(2) * 10
    rospy.loginfo("Getting random goal [{},{}]".format(x,y))
    action_goal.target_pose.header.frame_id = "odom"
    action_goal.target_pose.pose.position.x = x
    action_goal.target_pose.pose.position.y = y
    action_goal.target_pose.pose.orientation.z = 1.0
    
def doneCallback(state, result):
    if (state != GoalStatus.SUCCEEDED):
        rospy.logwarn("Action failed")
        return
    elif (state == GoalStatus.SUCCEEDED):
        rospy.loginfo("Action sucessful!")
    rospy.sleep(2)

    getRandomGoal(action_goal)
    client.send_goal(action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rospy.loginfo("Action goal: %s" % (str(action_goal)))
    return
    
if __name__ == "__main__":
    rospy.init_node("move base test script")
    rospy.loginfo("Connecting to %s..." % ACTION_NAME)
    client.wait_for_server()

    getRandomGoal(action_goal)

    rospy.loginfo("Sending goal...")
    client.send_goal(action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rospy.loginfo("Action goal: %s" % (str(action_goal)))

    rospy.spin()
