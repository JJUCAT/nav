#include <navit_msgs/SimpleNavigatorAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<navit_msgs::SimpleNavigatorAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigator_client");
  Client client("simple_navigator", true); // true -> don't need ros::spin()
  client.waitForServer();
  navit_msgs::SimpleNavigatorGoal goal;
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
      ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
  }
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");


  return 0;
}