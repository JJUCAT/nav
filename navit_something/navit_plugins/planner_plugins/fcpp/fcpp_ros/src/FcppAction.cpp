/*
 * FcppAction.cpp
 *
 *  Created on: 2023年6月5日
 *      Author: yjh
 */

#include "fcpp_ros/FcppAction.h"

namespace fcpp_ros {

void FcppAction::initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) {
	ros::NodeHandle pnh("~/"+name);
	// TODO Auto-generated constructor stub
	pnh.param("plan_type", plan_type, 0);
	pnh.param("sweep_step", sweep_step, 0.5);
	// todo add other parameter
	plan_ = new fcpp_ros::TopologyPlan(sweep_step);
	// register the done api
	is_init = false;
	path_pub_ = pnh.advertise<nav_msgs::Path>("edge_path", 1, true);
}

// FcppAction::~FcppAction() {
// 	// TODO Auto-generated destructor stub
// 	if (plan_)
// 		delete plan_;
// }

 bool FcppAction::makePlan(const geometry_msgs::Polygon& coverage_area,
                           const std::vector<geometry_msgs::Polygon>& holes,
                           const geometry_msgs::Pose& start,
                           const geometry_msgs::Pose& end,
                           std::vector<nav_msgs::Path>& coverage_paths,
                           std::vector<nav_msgs::Path>& contour_paths) {
	nav_msgs::Path path;
	path.header.frame_id = "map";
	geometry_msgs::PoseStamped pose_stamped;

	// pub
	for (int i = 0; i < coverage_area.points.size(); ++i) {
		geometry_msgs::PoseStamped pose;

        pose.header.frame_id = "map";
        pose.pose.position.x = coverage_area.points[i].x;
        pose.pose.position.y = coverage_area.points[i].y;
        pose.pose.position.z = coverage_area.points[i].z;

		pose.pose.orientation.w = 1;

        path.poses.push_back(pose);
	}
	path_pub_.publish(path);

	std::vector<geometry_msgs::Point> outer(coverage_area.points.size());
	for (unsigned int i = 0; i != coverage_area.points.size(); ++i) {
		outer[i].x = coverage_area.points[i].x;
		outer[i].y = coverage_area.points[i].y;
	}

	std::vector<std::vector<geometry_msgs::Point>> inner(holes.size());
	// May be a bug here(czk)
	for (unsigned int i = 0; i != holes.size(); ++i) {
		inner[i].resize(holes[i].points.size());
		for (unsigned int j = 0; j < holes[i].points.size(); ++j) {
			inner[i][j].x = holes[i].points[j].x;
			inner[i][j].y = holes[i].points[j].y;
		}
	}

	if (!is_init) {
		is_init = true;
		plan_->Init(outer, inner);
	} else {
		plan_->UpdateMap(outer, inner);
	}
	bool res = false;

	try{
	    res =plan_->Plan(start.position.x, start.position.y, plan_type);
	} catch(...){
		ROS_ERROR("plan error");
		return false;
	}

	if (res) {
		std::vector<polygon_coverage_planning::Point> points;
		plan_->GetPoints(points);
		nav_msgs::Path temp_path;
		points.erase(points.begin());

		for (auto point_it = points.begin(); point_it != points.end();
				++point_it) {
			if (!point_it->is_transfer_point) {
				geometry_msgs::PoseStamped p;
				p.pose.position.x = point_it->x_;
				p.pose.position.y = point_it->y_;
				temp_path.poses.push_back(p);
				if (std::next(point_it) == points.end()) {
					coverage_paths.push_back(temp_path); //for last path
				}
			} else {
				if(!temp_path.poses.empty()){
					coverage_paths.push_back(temp_path);
				}
				temp_path.poses.clear();
				geometry_msgs::PoseStamped p;
				p.pose.position.x = point_it->x_;
				p.pose.position.y = point_it->y_;
				temp_path.poses.push_back(p);
			}
		}
		//todo merge the trajectory
		for (auto path_it = coverage_paths.begin();
				path_it != coverage_paths.end(); ++path_it) {
			if (path_it->poses.size() < 1) {
				continue;
			}
			if (std::next(path_it) == coverage_paths.end()
					|| std::next(path_it)->poses.size() < 2) {
				continue;
			}
			geometry_msgs::PoseStamped line_start, line_end, p;
			p = path_it->poses.back();
			line_start = std::next(path_it)->poses.front();
			line_end = *std::next(std::next(path_it)->poses.begin());
			if(comupteNeedMerge(line_start, line_end,p)){
				ROS_INFO("merge rigger");
			}
		}
		//Add frame id
		for (int i = 0; i < coverage_paths.size(); ++i ){
			coverage_paths[i].header.frame_id = "map";
			coverage_paths[i] = resamplePath(coverage_paths[i], 0.05);
			for(int j = 0; j < coverage_paths[i].poses.size(); ++j) {
				coverage_paths[i].poses[j].header.frame_id = "map";
				coverage_paths[i].poses[j].pose.orientation.x = 0;
				coverage_paths[i].poses[j].pose.orientation.y = 0;
				coverage_paths[i].poses[j].pose.orientation.z = 0;
				coverage_paths[i].poses[j].pose.orientation.w = 1;
			}
		}

		return true;
	} else {
		return false;
	}
}

double FcppAction::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
	return sqrt(pow(p2.pose.position.x - p1.pose.position.x, 2) +
				pow(p2.pose.position.y - p1.pose.position.y, 2) +
				pow(p2.pose.position.z - p1.pose.position.z, 2));
}

geometry_msgs::PoseStamped FcppAction::interpolate(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2, double ratio) {
	geometry_msgs::PoseStamped interpolated_pose;

	interpolated_pose.pose.position.x = (1 - ratio) * p1.pose.position.x + ratio * p2.pose.position.x;
	interpolated_pose.pose.position.y = (1 - ratio) * p1.pose.position.y + ratio * p2.pose.position.y;
	interpolated_pose.pose.position.z = (1 - ratio) * p1.pose.position.z + ratio * p2.pose.position.z;

	interpolated_pose.pose.orientation = p2.pose.orientation;
	return interpolated_pose;
}
nav_msgs::Path FcppAction::resamplePath(const nav_msgs::Path& input_path, double desired_spacing) {
    nav_msgs::Path output_path;
    output_path.header.frame_id = "map";

    if (input_path.poses.size() < 2 || desired_spacing < 0.0) {
      ROS_ERROR("Input size is less than 2");
      return output_path;
    }

    output_path.poses.push_back(input_path.poses[0]);
    double excess = 0.0;

    size_t i = 1;
    geometry_msgs::PoseStamped last_added = input_path.poses[0];

    while (i < input_path.poses.size()) {
        double d = distance(last_added, input_path.poses[i]);

        if (d < desired_spacing) {
            output_path.poses.push_back(input_path.poses[i]);
            last_added = input_path.poses[i];
            ++i;
        } else if (d + excess < desired_spacing) {
            excess += d;
            ++i;
        } else {
            double ratio = (desired_spacing - excess) / d;
            geometry_msgs::PoseStamped interpolated_pose = interpolate(last_added, input_path.poses[i], ratio);

            output_path.poses.push_back(interpolated_pose);
            last_added = interpolated_pose;

            if (ratio == 1.0) {
                ++i;
            }
            excess = 0.0;
        }
    }

    return output_path;
}
bool FcppAction::comupteNeedMerge(
		const geometry_msgs::PoseStamped &line_start,
		const geometry_msgs::PoseStamped &line_end,
		const geometry_msgs::PoseStamped &point) {
	// coumpute the distance between point and line
	const Eigen::Vector2d p0(line_start.pose.position.x, line_start.pose.position.y);
	const Eigen::Vector2d p1(line_end.pose.position.x,line_end.pose.position.y);
	const double lane_length = (p1 - p0).norm();
	const Eigen::Vector2d p_cur = Eigen::Vector2d(point.pose.position.x,point.pose.position.y);
	const Eigen::Vector2d pn = (p1 - p0) / lane_length;
	const Eigen::Vector2d p_l = p_cur - p0;
	const Eigen::Vector2d p_cros = p_l - p_l.dot(pn) * pn;
	ROS_INFO("p_cross is %f, pn is %f", p_cros.norm(),p_l.norm());
//	if(p_cros.norm()<0.1 && p_l.norm()<0.3){
//		return true;
//	}
	if(p_cros.norm()<0.1){
		return true;
	}
	return false;
}
} // namespace fcpp_ros
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fcpp_ros::FcppAction, navit_core::CoveragePlanner)
