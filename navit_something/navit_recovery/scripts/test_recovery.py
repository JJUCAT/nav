#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from navit_msgs.msg import BackUpAction, BackUpGoal, BackUpResult
from navit_msgs.msg import WaitAction, WaitGoal, WaitResult

import numpy as np

backup_client = actionlib.SimpleActionClient("/backup", BackUpAction)
backup_distance = [0.1, 0.2, 3, 2.5]
backup_speed = [0.1, 0.2, 0.3, 0.5]

wait_client = actionlib.SimpleActionClient("/wait", WaitAction)
wait_duration = [5, 7, 9 ,11]

wait_goal = WaitGoal()
backup_goal = BackUpGoal()

def getRandomGoal():
    rand_distance = np.random.random_integers(0,3)
    rand_speed = np.random.random_integers(0,3)
    backup_goal.backup_distance = backup_distance[rand_distance]
    backup_goal.speed = backup_speed[rand_speed]

    rand_wait = np.random.random_integers(0,3)
    wait_goal.time.data = rospy.Duration(wait_duration[rand_wait])

    rand_action = np.random.random_integers(0,1)
    goals = [backup_goal, wait_goal]
    clients = [backup_client, wait_client]
    action_goal = goals[rand_action]
    client = clients[rand_action]
    return client, action_goal
    
def doneCallback(state, result):
    if (state != GoalStatus.SUCCEEDED):
        rospy.logwarn("Action failed")
        return
    elif (state == GoalStatus.SUCCEEDED):
        rospy.loginfo("Action sucessful!")
    rospy.sleep(2)

    client, action_goal = getRandomGoal()
    client.send_goal(action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rospy.loginfo("Action goal: %s" % (str(action_goal)))
    return


if __name__ == "__main__":
    rospy.init_node("recovery_test_node")
    backup_client.wait_for_server()
    wait_client.wait_for_server()

    client, action_goal = getRandomGoal()

    rospy.loginfo("Sending goal...")
    client.send_goal(action_goal, done_cb=doneCallback, active_cb=None, feedback_cb=None)
    rospy.loginfo("Action goal: %s" % (str(action_goal)))

    rospy.spin()
