#!/usr/bin/env python
import rospy
import actionlib
from navit_msgs.msg import CoveragePathAction, CoveragePathGoal 
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import numpy as np

ACTION_NAME = "/coverage_path" 
client = actionlib.SimpleActionClient(ACTION_NAME, CoveragePathAction)
action_goal = CoveragePathGoal()
planner_plugins = ["test_1","test_2","test_3"]
x_offsets = [10,50,80]
y_offsets = [80,50,10]

def getRandomGoal(action_goal):
    rand_planner = np.random.random_integers(0,2)
    num_edge = 4
    [x,y] = np.random.randn(2,num_edge) * 10
    x = [0.5, 0.5, 28, 28]
    y = [0.5, 28, 28, 0.5]
    rospy.loginfo("Getting random goal [{},{}]".format(x,y))
    action_goal.planner_plugin = planner_plugins[rand_planner]
    action_goal.edge_path.poses.clear()
    action_goal.edge_path.header.frame_id = "map"

    for i in range(num_edge):
        p = PoseStamped()
        p.pose.position.x = x[i] + x_offsets[rand_planner]
        p.pose.position.y = y[i] + y_offsets[rand_planner]
        action_goal.edge_path.poses.append(p)
    
    p = PoseStamped()
    p.pose.position.x = x[0] + x_offsets[rand_planner]
    p.pose.position.y = y[0] + y_offsets[rand_planner]
    action_goal.edge_path.poses.append(p)

    map_pub.publish(action_goal.map)
    edge_pub.publish(action_goal.edge_path)
    return
    
def doneCallback(state, result):
    rospy.loginfo("planning is Done")
    rospy.sleep(5)
    getRandomGoal(action_goal)
    client.send_goal(action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rospy.loginfo("all done!")
    return
   
def mapCallback(msg):
    rospy.loginfo("map received!")
    action_goal.map.header = msg.header
    action_goal.map.info = msg.info
    action_goal.map.data = msg.data
    map_pub.publish(action_goal.map)
    return

if __name__ == "__main__":
    rospy.init_node("test_ccpp")
    rospy.loginfo("Connecting to %s..." % ACTION_NAME)
    client.wait_for_server()

    rospy.Subscriber("/map", OccupancyGrid, mapCallback)
    map_pub = rospy.Publisher("/test_ccpp_map",OccupancyGrid)
    edge_pub = rospy.Publisher("/test_ccpp_edge",Path)

    getRandomGoal(action_goal)

    rospy.loginfo("Sending ccpp goal...")
    client.send_goal(action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)

    rospy.spin()
