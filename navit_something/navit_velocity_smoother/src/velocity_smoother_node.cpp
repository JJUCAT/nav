#include <ros/ros.h>
#include <velocity_smoother.h>

using namespace navit_velocity_smoother;

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_smoother");
  ros::NodeHandle nh_p("~");
  std::string name = "velocity_smoother";

  std::shared_ptr<VelocitySmoother> vel_smoother_;

  std::string type;
  nh_p.getParam("type", type);

  if (type == "twist")
    vel_smoother_.reset(new TwistVelocitySmoother(name, nh_p));
  else if (type == "ackermann")
    vel_smoother_.reset(new AckermannVelocitySmoother(name, nh_p));
  else {
    ROS_ERROR("missing vel smoother type (twist or ackermann???)");
    return -1;
  }

  vel_smoother_->spin();

  return 0;
}
