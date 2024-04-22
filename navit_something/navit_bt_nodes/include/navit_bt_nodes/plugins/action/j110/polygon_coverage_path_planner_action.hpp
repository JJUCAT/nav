#ifndef PWH_COVERGE_PLANNER_ACTION_H_
#define PWH_COVERGE_PLANNER_ACTION_H_

#include <string>

#include <navit_msgs/CoveragePathOnPWHAction.h>
#include <geometry_msgs/PolygonStamped.h>
#include <navit_common/log.h>

#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "message_navit_map.pb.h"

namespace navit_bt_nodes
{

class PWHCoveragePlannerAction : public BT::RosActionNode<navit_msgs::CoveragePathOnPWHAction>
{
public:
    PWHCoveragePlannerAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
                            : RosActionNode<navit_msgs::CoveragePathOnPWHAction>(handle, name, conf) {
        ROS_INFO("PWHCoveragePlannerAction constructor");
    }

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<navit::protocol::map_info::MapArea>("task_polygon", "task polygon for coverage planner"),
            BT::InputPort<std::string>("plugin_name", "planner plugin name"),
            BT::OutputPort<nav_msgs::Path>("coverage_whole_path", "coverage path"),
            BT::OutputPort<bool>("path_updated", ""),
        });
    }


    bool on_first_tick() override {
        ROS_INFO("PWHCoveragePlannerAction first tick");
        navit::protocol::map_info::MapArea task_polygon;

        if (!getInput("task_polygon", task_polygon)) {
            NAVIT_ROS_ERROR_STREAM("task_polygon is not specified");
            return false;
        }

        if (task_polygon.type() == navit::protocol::map_info::MapArea::AREA_TYPE_FULL_COVERAGE) {
            geometry_msgs::Polygon coverage_area;
            geometry_msgs::Point32 point;

            for (const auto& pt : task_polygon.path()) {
                point.x = pt.x();
                point.y = pt.y();

                ROS_INFO("point.x: %f, point.y: %f", point.x, point.y);
                coverage_area.points.push_back(point);
            }
            if (coverage_area.points.end() != coverage_area.points.begin()) {
                coverage_area.points.push_back(coverage_area.points.front());
            }
            goal_.coverage_area = coverage_area;
        }

        if (task_polygon.type() == navit::protocol::map_info::MapArea::AREA_TYPE_FORBIDDEN_AREA) {
            geometry_msgs::Polygon hole;
            geometry_msgs::Point32 point;

            for (const auto& pt : task_polygon.path()) {
                point.x = pt.x();
                point.y = pt.y();
                hole.points.push_back(point);
            }
            goal_.holes.push_back(hole);
        }

        if (!getInput("plugin_name", goal_.plugin_name)) {
            NAVIT_ROS_ERROR_STREAM("plugin_name is not specified");
            return false;
        }

        // get pose from tf
        // get pose from tf baselink
        try {
            tf::StampedTransform transform;
            listener.lookupTransform("map", "base_link", ros::Time(0), transform);
            goal_.start.position.x = transform.getOrigin().x();
            goal_.start.position.y = transform.getOrigin().y();
            goal_.start.position.z = transform.getOrigin().z();
            goal_.start.orientation.x = transform.getRotation().x();
            goal_.start.orientation.y = transform.getRotation().y();
            goal_.start.orientation.z = transform.getRotation().z();
            goal_.start.orientation.w = transform.getRotation().w();
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return false;
        }
        goal_.end = goal_.start;
        // if (!getInput("robot_pose", goal_.start)) {
        //     return false;
        // }

        return true;
    }

    BT::NodeStatus onResult(const ResultType& res) override {
        // if (res.error_code == navit_msgs::CoveragePathOnPWHResult::STATUS_OK) {

        // } else {
        //     return BT::NodeStatus::FAILURE;
        // }
        // return BT::NodeStatus::RUNNING;


        nav_msgs::Path coverage_whole_path;
        coverage_whole_path.header.stamp = ros::Time::now();
        coverage_whole_path.header.frame_id = "map";
        coverage_whole_path.header.seq = 0;
        for (auto& path : res.coverage_paths) {
            coverage_whole_path.poses.insert(coverage_whole_path.poses.end(), path.poses.begin(), path.poses.end());
        }
        // coverage_whole_path.poses.erase(coverage_whole_path.poses.begin());
        // coverage_whole_path.poses.pop_back();
        // coverage_whole_path = resamplePath(coverage_whole_path, 0.05);

        setOutput("coverage_whole_path", coverage_whole_path);
        setOutput("path_updated", true);
        return BT::NodeStatus::SUCCESS;

    }

    double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
	return sqrt(pow(p2.pose.position.x - p1.pose.position.x, 2) +
				pow(p2.pose.position.y - p1.pose.position.y, 2) +
				pow(p2.pose.position.z - p1.pose.position.z, 2));
    }

    geometry_msgs::PoseStamped interpolate(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2, double ratio) {
        geometry_msgs::PoseStamped interpolated_pose;

        interpolated_pose.pose.position.x = (1 - ratio) * p1.pose.position.x + ratio * p2.pose.position.x;
        interpolated_pose.pose.position.y = (1 - ratio) * p1.pose.position.y + ratio * p2.pose.position.y;
        interpolated_pose.pose.position.z = (1 - ratio) * p1.pose.position.z + ratio * p2.pose.position.z;

        interpolated_pose.pose.orientation = p2.pose.orientation;
        return interpolated_pose;
    }

    nav_msgs::Path resamplePath(const nav_msgs::Path& input_path, double desired_spacing) {
        nav_msgs::Path output_path;
        output_path.header.frame_id = "map";


        if(input_path.poses.size() < 2) {
            ROS_ERROR("Input size is less than 2");
            return output_path;
        }

        output_path.poses.push_back(input_path.poses[0]);
        double excess = 0.0;

        size_t i = 1;
        geometry_msgs::PoseStamped last_added = input_path.poses[0];

        while (i < input_path.poses.size()) {
            double d = distance(last_added, input_path.poses[i]);

            if (d + excess < desired_spacing) {
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
    private:
        tf::TransformListener listener;
};

}  // namespace navit_bt_nodes
#endif  // PWH_COVERGE_PLANNER_ACTION_H_
