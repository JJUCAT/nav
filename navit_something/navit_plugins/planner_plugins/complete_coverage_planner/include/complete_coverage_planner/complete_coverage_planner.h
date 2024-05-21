#ifndef CCPP_PLANNER_COMPLETE_COVERAGE_PLANNER_H
#define CCPP_PLANNER_COMPLETE_COVERAGE_PLANNER_H

#include <navit_core/base_global_planner.h>
#include <complete_coverage_planner/ccpp.h>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace complete_coverage_planner
{
    class CompleteCoveragePlanner : public navit_core::CoveragePlanner
    {
        public:
            CompleteCoveragePlanner():ccpp_ptr_(nullptr){}

            ~CompleteCoveragePlanner(){ccpp_ptr_.reset();}

            void initialize(const std::string& name) override;

            bool makePlan(const nav_msgs::OccupancyGrid& map,
                          const nav_msgs::Path& edge_path,
                          nav_msgs::Path& coverage_path, 
                          nav_msgs::Path& wall_path) override;

        private:
            std::shared_ptr<ccpp_planner::CCPP> ccpp_ptr_;
            cv::Mat mapToImage(const nav_msgs::OccupancyGrid& map);
            std::vector<ccpp_planner::MapPose> pathToMapPose(const nav_msgs::Path& path);
            nav_msgs::Path mapPoseToPath(const std::vector<ccpp_planner::MapPose>& poses);

            struct config
            {
                double map_resolution = 0.05;
                double coverage_resolution = 0.1;
                double robot_radius = 0.2; 
                double coverage_interval_distance = robot_radius * 2 * 2;
                double inflation_distance = robot_radius + 0.15;
                double path_eps = 1.0;
            } config_;

            nav_msgs::OccupancyGrid map_;
    };

}
#endif
