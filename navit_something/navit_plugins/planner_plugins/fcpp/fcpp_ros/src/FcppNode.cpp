/*
 * FcppNode.cpp
 *
 *  Created on: 2023年6月5日
 *      Author: yjh
 */

#include "fcpp_ros/FcppAction.h"

int main(int argc, char ** argv){
	ros::init(argc, argv, "fcpp_server_node");
	ros::NodeHandle n("~");
	// fcpp_ros::FcppAction ac_(n);
	ROS_INFO("the fcpp server run success");
	ros::spin();
	return 0;
}


