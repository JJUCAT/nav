/*
 * TopologyMap.cpp
 *
 *  Created on: 2023年6月5日
 *      Author: yjh
 */

#include "fcpp_ros/TopologyPlan.h"

namespace fcpp_ros {

TopologyPlan::TopologyPlan(double sweep_step) {
	// TODO configure by ros param
	config_.counter_clockwise = true; //逆时针
	config_.decompose_type = 0; // BCD 区域分解
	config_.sweep_step = sweep_step;
	planner_.init(config_);
	ROS_INFO("TopologyPlan init success");
}

TopologyPlan::~TopologyPlan() {
	result_.clear();
}

bool TopologyPlan::Init(std::vector<geometry_msgs::Point> &outer_boundary,
		std::vector<std::vector<geometry_msgs::Point>> &obst_boundary) {
	UpdateMap(outer_boundary, obst_boundary);
	// update the other parameter
	return true;
}

bool TopologyPlan::UpdateMap(std::vector<geometry_msgs::Point> &outer_boundary,
		std::vector<std::vector<geometry_msgs::Point>> &obst_boundary) {
	std::vector<polygon_coverage_planning::Point> outer_polygon;
	std::vector < std::vector
			< polygon_coverage_planning::Point
					>> innner_polygon(obst_boundary.size());
	for (const auto &point : outer_boundary) {
		outer_polygon.push_back(
				polygon_coverage_planning::Point(point.x, point.y));
	}
	for (unsigned int i = 0; i < obst_boundary.size(); i++) {
		for (const auto &point : obst_boundary[i]) {
			innner_polygon[i].push_back(
					polygon_coverage_planning::Point(point.x, point.y));
		}
	}
	planner_.Update(outer_polygon, innner_polygon);
	return true;
}

//0--default(single row) 1--double row
bool TopologyPlan::Plan(float start_x, float start_y, int plan_type) {
	result_.clear();
	polygon_coverage_planning::Point start;
	start.x_ = start_x;
	start.y_ = start_y;
	start.is_transfer_point = true;
	bool res = false;
	if (plan_type == 0) {
		res = planner_.SolveSingleBow(start); // use the single row
	} else if (plan_type == 1) {
		res = planner_.SolveDoubleBow(start); // use the double row
	}
	if (res) {
		planner_.getWayPoint(result_);
		return true;
	} else {
		return false;
	}
}

void TopologyPlan::GetPoints(
		std::vector<polygon_coverage_planning::Point> &point) {
	point = result_;
}

} // namespace fcpp_ros
