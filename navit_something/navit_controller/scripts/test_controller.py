#!/usr/bin/env python3
import rospy
import actionlib
from navit_msgs.msg import ComputePathAction, ComputePathGoal, ComputePathResult
from navit_msgs.msg import FollowPathAction, FollowPathGoal, FollowPathFeedback
from geometry_msgs.msg import PoseStamped

import numpy as np

PLANNER_ACTION_NAME = "/compute_path" 
planner_client = actionlib.SimpleActionClient(PLANNER_ACTION_NAME, ComputePathAction)
planner_action_goal = ComputePathGoal()
planner_result = ComputePathResult()
planner_plugins = ["test_1", "test_2", "test_3",""]

CONTROLLER_ACTION_NAME = "/follow_path"
controller_client = actionlib.SimpleActionClient(CONTROLLER_ACTION_NAME, FollowPathAction)
controller_action_goal = FollowPathGoal()
controller_plugins = ["test_1", "test_2", "test_3",""]

goal_coords = [[15, 23],
               [20, 23],
               [25, 23]
                ]

def getRandomGoal():
    rand_planner = np.random.randint(len(planner_plugins))
    rand_goal = np.random.randint(len(goal_coords))
    planner_action_goal.planner_plugin = planner_plugins[rand_planner]
    [x,y] = goal_coords[rand_goal]
    planner_action_goal.goal.header.frame_id = "map"
    planner_action_goal.goal.header.stamp = rospy.Time.now()
    planner_action_goal.goal.pose.position.x = x
    planner_action_goal.goal.pose.position.y = y
    planner_action_goal.goal.pose.orientation.z = 1.0
    planner_action_goal.use_start = False
    goal_pub = rospy.Publisher("/test_goal", PoseStamped, queue_size = 1)
    goal_pub.publish(planner_action_goal.goal)



def getControllerGoal(state, result):
    rand_controller = np.random.randint(len(controller_plugins))
    controller_action_goal.controller_plugin = controller_plugins[rand_controller]
    controller_action_goal.path = planner_result.path

def controlDoneCallback(state, result):
    rospy.loginfo("control is done")
    rospy.loginfo("getting new plan")

    getRandomGoal()
    planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    
def doneCallback(state, result):
    rospy.loginfo("planning is Done")
    rospy.sleep(0.1)
    if (not result.path.poses):
        rospy.loginfo("failed!")
        getRandomGoal()
        planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rand_controller = np.random.randint(len(controller_plugins))
    controller_action_goal.controller_plugin = controller_plugins[0]
    controller_action_goal.path = result.path
    controller_client.send_goal(controller_action_goal, done_cb=controlDoneCallback, active_cb=None, feedback_cb=None)
    rospy.loginfo("all done!")
    return
    


if __name__ == "__main__":
    rospy.init_node("controller_test_node")
    planner_client.wait_for_server()
    controller_client.wait_for_server()

    getRandomGoal()

    rospy.loginfo("Sending dock goal...")
    planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)

    rospy.spin()
