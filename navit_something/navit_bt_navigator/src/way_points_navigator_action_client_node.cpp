#include <navit_msgs/WayPointsNavigatorAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<navit_msgs::WayPointsNavigatorAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "way_points_navigator_client");
  Client client("way_points_navigator", true); // true -> don't need ros::spin()
  client.waitForServer();
  navit_msgs::WayPointsNavigatorGoal goal;
  goal.start = true;
  // Fill in goal here
  client.sendGoal(goal);
//   client.waitForResult(ros::Duration(5.0));
//   if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     printf("Yay! The dishes are now clean");
//   printf("Current State: %s\n", client.getState().toString().c_str());

  while(ros::ok() && client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
      client.waitForResult(ros::Duration(5.0));
      ROS_INFO("WayPointsNavigatorAction Client: current state is [%s]\n", client.getState().toString().c_str());

      // if (client.getState() == actionlib::SimpleClientGoalState::ABORTED)
      if (client.getState() != actionlib::SimpleClientGoalState::ACTIVE)
      {
        client.waitForResult(ros::Duration(5.0));
        client.cancelAllGoals();
        client.sendGoal(goal);
      }      
  }
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");


  return 0;
}