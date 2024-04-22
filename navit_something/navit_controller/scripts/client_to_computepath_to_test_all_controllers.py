#!/usr/bin/env python3
import rospy
import actionlib
from navit_msgs.msg import ComputePathAction, ComputePathGoal,ComputePathResult
from navit_msgs.msg import FollowPathAction, FollowPathGoal, FollowPathFeedback
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

ACTION_NAME = "/compute_path" 
planner_client = actionlib.SimpleActionClient(ACTION_NAME, ComputePathAction)
planner_action_goal = ComputePathGoal()
planner_plugins = ["test_1"]
plan_map = OccupancyGrid()
# goal_coords = [[243,372],
#             #    [267,345],
#             #    [218,344],
#                [30,375]]
goal_coords = [ [45,30],
               [44,29],
               [43,28],
              ]
planner_result=ComputePathResult

CONTROLLER_ACTION_NAME = "/follow_path"
controller_client = actionlib.SimpleActionClient(CONTROLLER_ACTION_NAME, FollowPathAction)
controller_action_goal = FollowPathGoal()
controller_plugins = ["test_1", "test_2","test_3","test_4"]
plugin_num =0
coords_num =0

def getRandomGoal(planner_action_goal):
    global coords_num
    if coords_num > 2:
            coords_num=0
    rand_coord = np.random.randint(len(goal_coords))
    # get random coords on map
    x = goal_coords[coords_num][0]
    y = goal_coords[coords_num][1]
    coords_num +=1

    rospy.loginfo("Getting random goal [{},{}]".format(x,y))
    planner_action_goal.planner_plugin = planner_plugins[0]
    planner_action_goal.goal.header.frame_id = "map"
    planner_action_goal.goal.pose.position.x = x
    planner_action_goal.goal.pose.position.y = y
    planner_action_goal.goal.pose.orientation.z = 1.0
    planner_action_goal.use_start = False
    goal_pub = rospy.Publisher("/test_goal", PoseStamped, queue_size = 1)
    goal_pub.publish(planner_action_goal.goal)
    





def controlDoneCallback(state, result):
    rospy.loginfo("control is done")
    init_pose=PoseWithCovarianceStamped()
    init_pose.header.frame_id="map"
    init_pose.pose.pose.position.x=43
    init_pose.pose.pose.position.y=28
    init_pose.pose.pose.position.z=0
    init_pose.pose.pose.orientation.w=1
    init_pose.pose.covariance[0]=0.25
    init_pose.pose.covariance[7]=0.25
    init_pose.pose.covariance[35]=0.0685
    rospy.loginfo("controlDoneCallback done!")
    getRandomGoal(planner_action_goal)
    global plugin_num
    if plugin_num < (len(controller_plugins)-1):
        plugin_num +=1
        rospy.loginfo("Sending new goal...")
        planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    else:
        rospy.loginfo("all controllers test done........")
        re_=Bool()
        re_.data=True
        all_controllers_tests_done_pub.publish(re_)
        exit


    
def doneCallback(state, result):
    rospy.loginfo("planning is Done")
    rospy.sleep(0.1)
    if (not result.path_found):
        rospy.loginfo("planning failed!")
        getRandomGoal(planner_action_goal)
        planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    global plugin_num
    # if plugin_num > (len(controller_plugins)-1):
    #         plugin_num=0
    controller_action_goal.controller_plugin = controller_plugins[plugin_num]
    rospy.loginfo("loading controller_plugins...%s",controller_plugins[plugin_num]) 
    controller_action_goal.path=result.path
    controller_client.send_goal(controller_action_goal, done_cb=controlDoneCallback, active_cb=None, feedback_cb=None)
    controller_goal_pub.publish(controller_action_goal.path)
    # plugin_num +=1
    rospy.loginfo("doneCallback done!")
    return

if __name__ == "__main__":
    rospy.init_node("compute_path_follow_test")
    # result_pub = rospy.Publisher("/action_succeed",Bool,queue_size=5)

    rospy.wait_for_service("/static_map")
    get_map = rospy.ServiceProxy("/static_map", GetMap)
    get_map_resp = get_map()
    plan_map = get_map_resp.map
    rospy.loginfo("planning map is received...")
    localization_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size = 1)

    planner_client.wait_for_server()
    
    init_pose=PoseWithCovarianceStamped()
    init_pose.header.frame_id="map"
    init_pose.pose.pose.position.x=43
    init_pose.pose.pose.position.y=28
    init_pose.pose.pose.position.z=0
    init_pose.pose.pose.orientation.w=1
    init_pose.pose.covariance[0]=0.25
    init_pose.pose.covariance[7]=0.25
    init_pose.pose.covariance[35]=0.0685
    localization_pub.publish(init_pose)
    controller_client.wait_for_server()

    controller_goal_pub =rospy.Publisher("/follow_path",Path,queue_size=5)
    all_controllers_tests_done_pub =rospy.Publisher("/all_controllers_tests_done",Bool,queue_size=5)
    rospy.sleep(1.0)
    getRandomGoal(planner_action_goal)
    rospy.loginfo("Sending new goal...")
    planner_client.send_goal(planner_action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)

    rospy.spin()
