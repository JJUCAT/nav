#include <navit_localization/navit_localization_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navit_localization_server");
  ros::NodeHandle nh;

  navit_localization::LocalizationServer localization_server(nh);

  ros::spin();

  return 0;
}
