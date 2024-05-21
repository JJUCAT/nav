/*
 * TopologyMap.h
 *
 *  Created on: 2023年6月5日
 *      Author: yjh
 */

#ifndef FCPP_ROS_INCLUDE_FCPP_ROS_TOPOLOGYPLAN_H_
#define FCPP_ROS_INCLUDE_FCPP_ROS_TOPOLOGYPLAN_H_

#include <fcpp_algorithm/CoveragePlanner.h>
#include "geometry_msgs/Point.h"

#include <vector>

#include <ros/ros.h>

namespace fcpp_ros {

class TopologyPlan {
public:
	explicit TopologyPlan(double sweep_step = 0.2);

	virtual ~TopologyPlan();
	// use the map as the input
	bool Init(std::vector<geometry_msgs::Point> &outer_boundary,
			std::vector<std::vector<geometry_msgs::Point>> &obst_boundary);
	//
	bool UpdateMap(std::vector<geometry_msgs::Point> &outer_boundary,
			std::vector<std::vector<geometry_msgs::Point>> &obst_boundary);

	//0--default(single row) 1--double row
	bool Plan(float start_x, float start_y, int plan_type = 0);
	//
	void GetPoints(std::vector<polygon_coverage_planning::Point> &point);
private:
	polygon_coverage_planning::CoveragePlanner planner_;
	polygon_coverage_planning::CoveragePlannerConfig config_;
	std::vector<polygon_coverage_planning::Point> result_;
};

} // namespace fcpp_ros

#endif /* FCPP_ROS_INCLUDE_FCPP_ROS_TOPOLOGYPLAN_H_ */
