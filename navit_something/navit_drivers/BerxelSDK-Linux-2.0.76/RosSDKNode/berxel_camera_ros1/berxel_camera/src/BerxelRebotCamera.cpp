#include "BerxelHawkCamera.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "berxel_camera");
	ros::NodeHandle node;
	
	BerxelHawkCamera berxelCamera(node);
    int32_t ret = berxelCamera.initBerxelCamera();
    if (ret != 0)
    {
        ROS_ERROR("Init Berxel Camera Failed");
        return -1;
    }

    ros::spin();
	return 0;
}

