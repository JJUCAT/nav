/*
 * FcppAction.h
 *
 *  Created on: 2023年6月5日
 *      Author: yjh
 */

#ifndef FCPP_ROS_INCLUDE_FCPP_ROS__FCPPACTION_H_
#define FCPP_ROS_INCLUDE_FCPP_ROS__FCPPACTION_H_

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "dynamic_reconfigure/server.h"
#include <Eigen/Dense>
#include <navit_core/base_global_planner.h>
// add the message
#include <navit_msgs/CoveragePathOnPWHAction.h>

#include "fcpp_ros/TopologyPlan.h"

namespace fcpp_ros {

class FcppAction : public navit_core::CoveragePlanner {
public:
	FcppAction() {};
	~FcppAction() {};
	void initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override;
	bool makePlan(const nav_msgs::Path& edge_path, const geometry_msgs::Pose& start_pose, nav_msgs::Path& coverage_path) override {return true;} ;
	bool makePlan(const geometry_msgs::Polygon& coverage_area,
                  const std::vector<geometry_msgs::Polygon>& holes,
                  const geometry_msgs::Pose& start, 
                  const geometry_msgs::Pose& end,
                  std::vector<nav_msgs::Path>& coverage_paths,
                  std::vector<nav_msgs::Path>& contour_paths);
private:
	bool comupteNeedMerge(const geometry_msgs::PoseStamped &line_start,
			const geometry_msgs::PoseStamped &line_end,
			const geometry_msgs::PoseStamped &point);
	double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
	geometry_msgs::PoseStamped interpolate(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2, double ratio);
	nav_msgs::Path resamplePath(const nav_msgs::Path& input_path, double desired_spacing);

	
protected:
	fcpp_ros::TopologyPlan *plan_;
//  feedback
//	TODO ADD dynamic config
//	dynamic_reconfigure::Server<> dy_;
	int plan_type;
	bool is_init;
	double sweep_step;
	ros::Publisher path_pub_;
};

} // namespace fcpp_ros
#endif /* FCPP_ROS_INCLUDE_FCPPACTION_H_ */
