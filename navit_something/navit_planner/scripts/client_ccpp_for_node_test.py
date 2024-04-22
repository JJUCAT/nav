#!/usr/bin/env python3
import rospy
import actionlib
from navit_msgs.msg import CoveragePathAction, CoveragePathGoal 
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np

from navit_msgs.msg import SmoothPathAction, SmoothPathGoal

ACTION_NAME = "/coverage_path" 
client = actionlib.SimpleActionClient(ACTION_NAME, CoveragePathAction)
action_goal = CoveragePathGoal()

ACTION_NAME1 = "/smooth_path" 
smooth_path_client = actionlib.SimpleActionClient(ACTION_NAME1, SmoothPathAction)
smooth_path_action_goal = SmoothPathGoal()

def sendActionGoal(action_goal):
    rand_planner = np.random.random_integers(0,2)
    num_edge = 5
    # [x,y] = np.random.randn(2,num_edge) * 10
    x = [-16.939104, -18.329, 22.82651, 20.7898,-16.939104]
    y = [17.2233581, -13.694, -11.0486, 18.0486, 17.2233]
    
    rospy.loginfo("Getting random goal [{},{}]".format(x,y))
    action_goal.planner_plugin = "test_1"
    action_goal.edge_path.poses.clear()
    action_goal.edge_path.header.frame_id = "map"

    for i in range(num_edge):
        p = PoseStamped()
        p.pose.position.x = x[i] 
        p.pose.position.y = y[i] 
        action_goal.edge_path.poses.append(p)
    
    edge_pub.publish(action_goal.edge_path)
    return

def smoothPathDoneCallback(state, result):
    result.smooth_path.header.frame_id = "map"
    smooth_path_pub.publish(result.smooth_path)
    smooth_path_result_pub.publish(result.smooth_success)
    rospy.loginfo("smoothPathDoneCallback is Done")
    return
    
def doneCallback(state, result):
    rospy.loginfo("planning is Done")
    rospy.loginfo("length of result is %d",len(result.coverage_path.poses))
    cov_path=Path()
    cov_path=result.coverage_path
    cov_path.header.frame_id="map"

    if result.path_found and len(result.coverage_path.poses)>0 and len(result.wall_path.poses)>0:
        result_pub.publish(result.path_found)
        coverage_path_pub.publish(cov_path)
        wall_path_pub.publish(result.wall_path)
    smooth_path_action_goal.smooth_path_plugin = "test_4"
    smooth_path_action_goal.rough_path.header.frame_id = "map"
    smooth_path_action_goal.rough_path = result.wall_path
    rospy.loginfo("Sending smooth_path.rough_path...")
    smooth_path_client.send_goal(smooth_path_action_goal, done_cb=smoothPathDoneCallback, active_cb=None, feedback_cb=None)
    return



def mapCallback(msg):
    rospy.loginfo("map received!")
    action_goal.map.header = msg.header
    action_goal.map.info = msg.info
    action_goal.map.data = msg.data
    return

if __name__ == "__main__":
    rospy.init_node("test_ccpp and smoth_path")
    rospy.loginfo("Connecting to %s..." % ACTION_NAME)
    client.wait_for_server()

    rospy.Subscriber("/map", OccupancyGrid, mapCallback)
    result_pub = rospy.Publisher("/action_succeed",Bool, queue_size = 1)
    smooth_path_result_pub = rospy.Publisher("/smooth_path_action_succeed",Bool, queue_size = 1)
    edge_pub = rospy.Publisher("/test_ccpp/edge_path",Path, queue_size = 1)
    smooth_path_pub = rospy.Publisher("/smooth_path",Path, queue_size = 1)
    coverage_path_pub = rospy.Publisher("/test_ccpp/coverage_path",Path, queue_size = 1)
    wall_path_pub = rospy.Publisher("/test_ccpp/wall_path",Path, queue_size = 1)
    rospy.sleep(5)
    sendActionGoal(action_goal)
    rospy.loginfo("Sending ccpp goal...")
    client.send_goal(action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rospy.spin()
