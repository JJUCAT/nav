#!/usr/bin/env python3
import rospy
import actionlib
from navit_msgs.msg import ApproachDockAction, ApproachDockGoal, FinalDockAction, FinalDockGoal

ACTION_NAME = ["/approach_dock", "/final_dock"]
client = actionlib.SimpleActionClient(ACTION_NAME[0], ApproachDockAction)
final_dock_client = actionlib.SimpleActionClient(ACTION_NAME[1], FinalDockAction)
goal = ApproachDockGoal()
goal.controller_plugin_name = "FgController"
goal.perception_plugin_name = "fake_perception"
goal.expected_dock_pose.header.frame_id = "odom"
goal.expected_dock_pose.pose.position.x = -3.0
goal.expected_dock_pose.pose.position.y = 0.25
goal.expected_dock_pose.pose.orientation.w = 1.0
goal.pre_charging_pose = goal.expected_dock_pose

goal_ = FinalDockGoal()
goal_.dock_pose.header.frame_id = "odom"
goal_.dock_pose.pose.position.x = -3.5
goal_.dock_pose.pose.position.y = 0.25
goal_.dock_pose.pose.orientation.w = 1.0
goal_.rotate_in_place = False

    
def doneCallback(state, result):
    rospy.loginfo("approach dock is Done")
    final_dock_client.send_goal_and_wait(goal_)
    rospy.loginfo("all done!")
    return
    


if __name__ == "__main__":
    rospy.init_node("dock_script")
    rospy.loginfo("Connecting to %s..." % ACTION_NAME[0])
    client.wait_for_server()
    rospy.loginfo("Connecting to %s..." % ACTION_NAME[1])
    final_dock_client.wait_for_server()

    rospy.loginfo("Sending dock goal...")
    client.send_goal(goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)

    rospy.spin()
