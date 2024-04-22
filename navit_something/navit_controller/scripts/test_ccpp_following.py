#!/usr/bin/env python3
import rospy
import actionlib
from navit_msgs.msg import CoveragePathAction, CoveragePathGoal, CoveragePathResult
from navit_msgs.msg import FollowPathAction, FollowPathGoal, FollowPathFeedback
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

PLANNER_ACTION_NAME = "/coverage_path" 
planner_client = actionlib.SimpleActionClient(PLANNER_ACTION_NAME, CoveragePathAction)
planner_action_goal = CoveragePathGoal()
planner_result = CoveragePathResult()
planner_plugins = ["test_1","test_2","test_3"]
x_offsets = [0,0,0]
y_offsets = [0,0,0]

CONTROLLER_ACTION_NAME = "/follow_path"
controller_client = actionlib.SimpleActionClient(CONTROLLER_ACTION_NAME, FollowPathAction)
controller_action_goal = FollowPathGoal()
controller_plugins = ["test_1", "test_2"]

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
    init_pose=PoseWithCovarianceStamped()
    init_pose.header.frame_id="map"
    init_pose.pose.pose.position.x=10
    init_pose.pose.pose.position.y=10
    init_pose.pose.pose.position.z=0
    init_pose.pose.pose.orientation.w=1
    init_pose.pose.covariance[0]=0.25
    init_pose.pose.covariance[7]=0.25
    init_pose.pose.covariance[35]=0.06853892
    localization_pub.publish(init_pose)
    rospy.sleep(5.0)
    if (not result.coverage_path.poses):
        rospy.loginfo("failed!")
        getRandomGoal()
        planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    cov_path=Path()
    cov_path=result.coverage_path
    cov_path.header.frame_id="map"
    for i in range(len(cov_path.poses)):
        cov_path.poses[i].header.frame_id="map"
        
    rand_controller = np.random.randint(len(controller_plugins))
    controller_action_goal.controller_plugin = controller_plugins[rand_controller]
    controller_action_goal.path = cov_path

    controller_client.send_goal(controller_action_goal, done_cb=controlDoneCallback, active_cb=None, feedback_cb=None)
    rospy.loginfo("all done!")
    return
    
if __name__ == "__main__":
    rospy.init_node("ccpp_following_test")
    map_pub = rospy.Publisher("/test_ccpp_map",OccupancyGrid)
    edge_pub = rospy.Publisher("/test_ccpp_edge",Path)
    localization_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped)
    
    rospy.wait_for_service("/static_map")
    get_map = rospy.ServiceProxy("/static_map", GetMap)
    get_map_resp = get_map()
    planner_action_goal.map = get_map_resp.map
    rospy.loginfo("planning map is received...")

    planner_client.wait_for_server()
    controller_client.wait_for_server()

    getRandomGoal()

    rospy.loginfo("Sending ccpp goal...")
    planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rospy.spin()
