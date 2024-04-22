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
goal_coords = [[243,372],
               [267,345],
               [218,344],
               [30,375]]

def getRandomGoal(action_goal):
    # pick random planner
    rand_planner = np.random.randint(len(planner_plugins))

    rand_coord = np.random.randint(len(goal_coords))
    # get random coords on map
    x = goal_coords[rand_coord][0]
    y = goal_coords[rand_coord][1]

    rospy.loginfo("Getting random goal [{},{}]".format(x,y))
    action_goal.planner_plugin = planner_plugins[rand_planner]
    action_goal.goal.header.frame_id = "map"
    action_goal.goal.pose.position.x = x
    action_goal.goal.pose.position.y = y
    action_goal.goal.pose.orientation.z = 1.0
    action_goal.use_start = False
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
