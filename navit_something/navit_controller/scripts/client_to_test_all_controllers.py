#!/usr/bin/env python3
import rospy
import actionlib
from navit_msgs.msg import CoveragePathAction, CoveragePathGoal, CoveragePathResult
from navit_msgs.msg import FollowPathAction, FollowPathGoal, FollowPathFeedback
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import Bool

PLANNER_ACTION_NAME = "/coverage_path" 
planner_client = actionlib.SimpleActionClient(PLANNER_ACTION_NAME, CoveragePathAction)
planner_action_goal = CoveragePathGoal()
planner_result = CoveragePathResult()
planner_plugins = ["test_1","test_2","test_3",""]
x_offsets = [0,0,0]
y_offsets = [0,0,0]

CONTROLLER_ACTION_NAME = "/follow_path"
controller_client = actionlib.SimpleActionClient(CONTROLLER_ACTION_NAME, FollowPathAction)
controller_action_goal = FollowPathGoal()
controller_plugins = ["test_1", "test_2","test_3","test_4",""]

def getRandomGoal():
    rand_planner = np.random.randint(len(planner_plugins))
    num_edge = 4
    [x,y] = np.random.randn(2,num_edge) * 10
    x = [0.5, 0.5, 28, 28]
    y = [0.5, 28, 28, 0.5]
    rospy.loginfo("Getting random goal [{},{}]".format(x,y))
    planner_action_goal.planner_plugin = planner_plugins[rand_planner]
    planner_action_goal.edge_path.poses.clear()
    planner_action_goal.edge_path.header.frame_id = "map"

    for i in range(num_edge):
        p = PoseStamped()
        p.pose.position.x = x[i] + x_offsets[rand_planner]
        p.pose.position.y = y[i] + y_offsets[rand_planner]
        planner_action_goal.edge_path.poses.append(p)
    
    p = PoseStamped()
    p.pose.position.x = x[0] + x_offsets[rand_planner]
    p.pose.position.y = y[0] + y_offsets[rand_planner]
    planner_action_goal.edge_path.poses.append(p)

    map_pub.publish(planner_action_goal.map)
    edge_pub.publish(planner_action_goal.edge_path)

    return


def controlDoneCallback(state, result):
    rospy.loginfo("control is done")
    rospy.loginfo("getting new plan")

    getRandomGoal()
    planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    
def doneCallback(state, result):
    rospy.loginfo("planning is Done")
    rospy.sleep(0.1)
    if (not result.coverage_path.poses):
        rospy.loginfo("failed!")
        getRandomGoal()
        planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    cov_path=Path()
    cov_path=result.coverage_path
    cov_path.header.frame_id="map"
    if result.path_found and len(result.coverage_path.poses)>0 and len(result.wall_path.poses)>0:
        result_pub.publish(result.path_found)
        coverage_path_pub.publish(cov_path)
        wall_path_pub.publish(result.wall_path)
    rand_controller = np.random.randint(len(controller_plugins))
    controller_action_goal.controller_plugin = controller_plugins[rand_controller]
    controller_action_goal.path = result.coverage_path
    controller_goal_pub.publish(controller_action_goal.path)

    controller_client.send_goal(controller_action_goal, done_cb=controlDoneCallback, active_cb=None, feedback_cb=None)
    rospy.loginfo("all done!")
    return

def mapCallback(msg):
    rospy.loginfo("map received!")
    planner_action_goal.map.header = msg.header
    planner_action_goal.map.info = msg.info
    planner_action_goal.map.data = msg.data
    return
    
if __name__ == "__main__":
    rospy.init_node("ccpp_following_test")
    map_pub = rospy.Publisher("/test_ccpp_map",OccupancyGrid,queue_size=5)
    edge_pub = rospy.Publisher("/test_ccpp_edge",Path,queue_size=5)
    coverage_path_pub = rospy.Publisher("/test_ccpp/coverage_path",Path,queue_size=5)
    wall_path_pub = rospy.Publisher("/test_ccpp/wall_path",Path,queue_size=5)
    result_pub = rospy.Publisher("/action_succeed",Bool,queue_size=5)

    rospy.wait_for_service("/static_map")
    get_map = rospy.ServiceProxy("/static_map", GetMap)
    get_map_resp = get_map()
    planner_action_goal.map = get_map_resp.map
    rospy.loginfo("planning map is received...")

    planner_client.wait_for_server()
    controller_client.wait_for_server()
    controller_goal_pub =rospy.Publisher("/follow_path",Path,queue_size=5)
    rospy.sleep(1.0)
    getRandomGoal()

    rospy.loginfo("Sending ccpp goal...")
    planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rospy.spin()
