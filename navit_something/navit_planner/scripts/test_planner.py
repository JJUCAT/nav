#!/usr/bin/env python
import rospy
import actionlib
from navit_msgs.msg import ComputePathAction, ComputePathGoal 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

import numpy as np

ACTION_NAME = "/compute_path" 
client = actionlib.SimpleActionClient(ACTION_NAME, ComputePathAction)
action_goal = ComputePathGoal()
planner_plugins = ["test_1","test_2","test_3","test_4"]
plan_map = OccupancyGrid()
start_coords = [[-43,-72],
               [67,45],
               [-10,-10],
               [30,37]]

goal_coords = [[243,372],
               [267,345],
               [218,344],
               [30,375]]

def getRandomGoal(action_goal):
    # pick random planner
    rand_planner = np.random.randint(len(planner_plugins))

    rand_start = np.random.randint(len(start_coords))
    rand_goal = np.random.randint(len(goal_coords))
    # get random coords on map
    start_x = start_coords[rand_start][0]
    start_y = start_coords[rand_start][1]
    goal_x = goal_coords[rand_goal][0]
    goal_y = goal_coords[rand_goal][1]

    rospy.loginfo("Getting random start [{},{}]".format(start_x,start_y))
    rospy.loginfo("Getting random goal [{},{}]".format(goal_x,goal_y))
    action_goal.planner_plugin = planner_plugins[rand_planner]
    action_goal.goal.header.frame_id = "map"
    action_goal.goal.pose.position.x = goal_x
    action_goal.goal.pose.position.y = goal_y
    action_goal.goal.pose.orientation.z = 1.0
    action_goal.start.header.frame_id = "map"
    action_goal.start.pose.position.x = start_x
    action_goal.start.pose.position.y = start_y
    action_goal.start.pose.orientation.z = 1.0
    action_goal.use_start = True
    start_pub = rospy.Publisher("/test_start", PoseStamped, queue_size = 1)
    start_pub.publish(action_goal.start)
    goal_pub = rospy.Publisher("/test_goal", PoseStamped, queue_size = 1)
    goal_pub.publish(action_goal.goal)

    
def doneCallback(state, result):
    rospy.loginfo("planning is Done")
    rospy.sleep(0.1)
    getRandomGoal(action_goal)
    client.send_goal(action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rospy.loginfo("all done!")
    return

if __name__ == "__main__":
    rospy.init_node("planner_test_script")
    rospy.loginfo("Connecting to %s..." % ACTION_NAME)

    rospy.wait_for_service("/static_map")
    get_map = rospy.ServiceProxy("/static_map", GetMap)
    get_map_resp = get_map()
    plan_map = get_map_resp.map
    rospy.loginfo("planning map is received...")
    client.wait_for_server()
   
    getRandomGoal(action_goal)

    rospy.loginfo("Sending goal...")
    client.send_goal(action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)


    rospy.spin()
