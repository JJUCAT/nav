#include <recovery_wait/wait.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(recovery_wait::RecoveryWait, navit_core::Recovery)

using namespace navit_recovery;
using namespace recovery_wait;

RecoveryWait::RecoveryWait(){}

void
RecoveryWait::loadParams(const std::string& name)
{
    ros::NodeHandle pnh("~/" + name);
    std::string test_param;
    pnh.param("test_param", test_param, test_param);
    ROS_INFO("name is %s test_param is %s", name.c_str(),test_param.c_str());
}

RecoveryWait::~RecoveryWait(){}

Status
RecoveryWait::onRun(const ActionGoal::ConstPtr& action_goal)
{
   end_ = ros::Time::now() + action_goal->time.data; 
   ROS_DEBUG("Wait time received is %f sec", action_goal->time.data.toSec());
   return Status::SUCCEEDED;
}

Status
RecoveryWait::onCycleUpdate()
{
   ros::Time current_time = ros::Time::now();
   ros::Duration left_time = end_ - current_time;

   ROS_DEBUG("====== left time is %f sec=======", left_time.toSec());

   feedback_.left_time.data = left_time;
   as_->publishFeedback(feedback_);

   if (left_time.toSec() > 0 )
       return Status::RUNNING;
   else
       return Status::SUCCEEDED;
}
