#!/usr/bin/env python3
import rospy
import actionlib
from navit_msgs.msg import CoveragePathAction, CoveragePathGoal, CoveragePathResult
from navit_msgs.msg import ComputePathAction, ComputePathGoal, ComputePathResult
from nav_msgs.msg import OccupancyGrid, Path
from navit_msgs.msg import FollowPathAction, FollowPathGoal
from geometry_msgs.msg import PoseStamped, PointStamped

PLANNER_ACTION_NAME = "/coverage_path"
ccpp_client = actionlib.SimpleActionClient(PLANNER_ACTION_NAME, CoveragePathAction)
ccpp_action_goal = CoveragePathGoal()
ccpp_result = CoveragePathResult()
ccpp_plugins = "ccpp"
ccpp_boundary_points_num = 0
get_map = False

PLANNER_ACTION_NAME = "/compute_path"
planner_client = actionlib.SimpleActionClient(PLANNER_ACTION_NAME, ComputePathAction)
planner_action_goal = ComputePathGoal()
planner_result = ComputePathResult()
planner_plugins = "topoint"

CONTROLLER_ACTION_NAME = "/follow_path"
controller_client = actionlib.SimpleActionClient(CONTROLLER_ACTION_NAME, FollowPathAction)
controller_action_goal = FollowPathGoal()
controller_plugins_edge = "ef"
# controller_plugins_edge = "mpc_avoidance"
controller_plugins_ccpp = "slp"
# controller_plugins_p2p = "mpc_avoidance"
controller_plugins_p2p = "pf"
# controller_plugins_p2p = "slp"

WALL_ACTION_NAME = "/follow_wall"
wall_client = actionlib.SimpleActionClient(WALL_ACTION_NAME, FollowPathAction)

ccpp_path_head_idx = 0 # 寻找可达的沿边起点
task_step = 0 # @0:空闲; @1:去沿边起点; @2:沿边; @3:去全覆盖起点; @4:全覆盖

def recvRvizPoint(msg):
    global ccpp_boundary_points_num
    global task_step
    rospy.loginfo("recv boundary point[%d]:[%.3f, %.3f]", ccpp_boundary_points_num, msg.point.x, msg.point.y)
    ccpp_action_goal.planner_plugin = ccpp_plugins
    ccpp_action_goal.edge_path.header.frame_id = "map"
    p = PoseStamped()
    p.pose.position.x = msg.point.x
    p.pose.position.y = msg.point.y
    ccpp_action_goal.edge_path.poses.append(p)
    ccpp_boundary_points_num += 1
    if ccpp_boundary_points_num == 4:
        tp = PoseStamped()
        tp.pose.position.x = ccpp_action_goal.edge_path.poses[0].pose.position.x
        tp.pose.position.y = ccpp_action_goal.edge_path.poses[0].pose.position.y
        ccpp_action_goal.edge_path.poses.append(tp)
    boundary_pub.publish(ccpp_action_goal.edge_path)

    if ccpp_boundary_points_num == 4:
      rospy.loginfo("get boundary")
      controller_action_goal.controller_plugin = controller_plugins_edge
      controller_action_goal.path = ccpp_action_goal.edge_path
      # controller_client.send_goal(controller_action_goal, done_cb=controlDoneCallback, active_cb=None, feedback_cb=None)
      wall_client.send_goal(controller_action_goal, done_cb=wallDoneCallback, active_cb=None, feedback_cb=None)
      ccpp_boundary_points_num = 0
      ccpp_action_goal.edge_path.poses.clear()


def mapCallback(msg):
    global get_map
    if get_map == False:
      ccpp_action_goal.map.header = msg.header
      ccpp_action_goal.map.info = msg.info
      ccpp_action_goal.map.data = msg.data
      map_pub.publish(msg)
      get_map = True
      

def ccppDoneCallback(state, result):
    global ccpp_path_head_idx
    global ccpp_result
    rospy.loginfo("ccpp is done")
    if result.path_found == True:
      rospy.loginfo("ccpp successed !")
      ccpp_result = result
      ccpp_path_head_idx = 0
      planner_action_goal.planner_plugin = planner_plugins
      planner_action_goal.goal.header.frame_id = "map"
      planner_action_goal.goal.header.stamp = rospy.Time.now()
      planner_action_goal.goal = ccpp_result.wall_path.poses[0]
      planner_action_goal.use_start = False
      planner_client.send_goal(planner_action_goal, done_cb=plannerDoneCallback, active_cb=None, feedback_cb=None)
    else:
      rospy.loginfo("ccpp failed !")

def wallDoneCallback(state, result):
    rospy.loginfo("wall is done")

def controlDoneCallback(state, result):
    global task_step
    global ccpp_result
    rospy.loginfo("control is done")
    if ccpp_result.path_found == True:
      if task_step == 1:
        task_step = 2
        rospy.loginfo("reach edge start point")
        controller_action_goal.controller_plugin = controller_plugins_edge
        controller_action_goal.path = ccpp_result.wall_path
        controller_client.send_goal(controller_action_goal, done_cb=controlDoneCallback, active_cb=None, feedback_cb=None)
      elif task_step == 2:
        task_step = 3
        rospy.loginfo("finish edge, ccpp size:%d", len(ccpp_result.coverage_path.poses))
        planner_action_goal.planner_plugin = planner_plugins
        planner_action_goal.goal.header.frame_id = "map"
        planner_action_goal.goal.header.stamp = rospy.Time.now()
        planner_action_goal.goal = ccpp_result.coverage_path.poses[0]
        planner_action_goal.use_start = False
        planner_client.send_goal(planner_action_goal, done_cb=plannerDoneCallback, active_cb=None, feedback_cb=None)
      elif task_step == 3:
        task_step = 4
        rospy.loginfo("reach ccpp start point")
        controller_action_goal.controller_plugin = controller_plugins_ccpp
        controller_action_goal.path = ccpp_result.coverage_path
        controller_client.send_goal(controller_action_goal, done_cb=controlDoneCallback, active_cb=None, feedback_cb=None)
      elif task_step == 4:
        task_step = 0
        ccpp_result.path_found = False
        rospy.loginfo("finish ccpp")



def plannerDoneCallback(state, result):
    global ccpp_path_head_idx
    global task_step
    global ccpp_result
    rospy.loginfo("planning is done")
    if result.path_found:
      rospy.loginfo("task step %d", task_step)
      if task_step <= 1:
        rospy.loginfo("plan to edge start point successed !")
        controller_action_goal.controller_plugin = controller_plugins_p2p
        controller_action_goal.path = result.path
        controller_client.send_goal(controller_action_goal, done_cb=controlDoneCallback, active_cb=None, feedback_cb=None)
      if task_step == 3:
        rospy.loginfo("plan to ccpp start point successed !")
        controller_action_goal.controller_plugin = controller_plugins_p2p
        controller_action_goal.path = result.path
        controller_client.send_goal(controller_action_goal, done_cb=controlDoneCallback, active_cb=None, feedback_cb=None)
    else:
      rospy.loginfo("planner failed!")
      if ccpp_result.path_found == True:
        if task_step == 1:
          rospy.loginfo("planner plan to edge start point failed!")
          head_step = 20
          if head_step > len(ccpp_result.wall_path.poses) - 1:
            head_step = len(ccpp_result.wall_path.poses) - 1
          head = ccpp_result.wall_path.poses[:head_step]
          del ccpp_result.wall_path.poses[:head_step]
          for p in head:
            ccpp_result.wall_path.poses.append(p)

          planner_action_goal.planner_plugin = planner_plugins
          planner_action_goal.goal.header.frame_id = "map"
          planner_action_goal.goal.header.stamp = rospy.Time.now()
          planner_action_goal.goal = ccpp_result.wall_path.poses[0]
          planner_action_goal.use_start = False
          planner_client.send_goal(planner_action_goal, done_cb=plannerDoneCallback, active_cb=None, feedback_cb=None)
        if task_step == 3:
          rospy.loginfo("planner plan to ccpp start point failed!")
          if ccpp_path_head_idx < len(ccpp_result.coverage_path.poses) - 1:
            ccpp_path_head_idx += 20
            rospy.loginfo("planner try ccpp next pose [%d]", ccpp_path_head_idx)
          else:
            ccpp_result.path_found = False
          planner_action_goal.planner_plugin = planner_plugins
          planner_action_goal.goal.header.frame_id = "map"
          planner_action_goal.goal.header.stamp = rospy.Time.now()
          planner_action_goal.goal = ccpp_result.coverage_path.poses[ccpp_path_head_idx]
          planner_action_goal.use_start = False
          planner_client.send_goal(planner_action_goal, done_cb=plannerDoneCallback, active_cb=None, feedback_cb=None)

def recv2DGoal(msg):
    rospy.loginfo("recv new goal:[%.3f, %.3f]", msg.pose.position.x, msg.pose.position.y)
    planner_action_goal.planner_plugin = planner_plugins
    planner_action_goal.goal = msg
    planner_action_goal.use_start = False
    planner_client.send_goal(planner_action_goal, done_cb=plannerDoneCallback, active_cb=None, feedback_cb=None)


if __name__ == "__main__":
    rospy.init_node("ccpp_test")
    ccpp_client.wait_for_server()
    planner_client.wait_for_server()
    controller_client.wait_for_server()

    rospy.Subscriber("/map", OccupancyGrid, mapCallback)
    map_pub = rospy.Publisher("/recv_map",OccupancyGrid)
    boundary_pub = rospy.Publisher("/ccpp_boundary",Path)
    rospy.Subscriber("/clicked_point", PointStamped, recvRvizPoint)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, recv2DGoal)

    rospy.loginfo("please add 4 points in rviz for ccpp boundary and start ccpp")
    rospy.spin()
