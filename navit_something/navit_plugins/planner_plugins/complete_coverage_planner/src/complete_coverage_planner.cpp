#include <complete_coverage_planner/complete_coverage_planner.h>

namespace complete_coverage_planner
{
    void CompleteCoveragePlanner::initialize(const std::string& name)
    {
        ros::NodeHandle pnh("~/"+name); 

        ccpp_planner::CcppConfig ccpp_cfg;

        pnh.param("map_resolution", ccpp_cfg.map_resolution, 0.05);
        pnh.param("coverage_resolution", ccpp_cfg.coverage_resolution, 0.1);
        pnh.param("robot_radius", ccpp_cfg.robot_radius, 0.2);
        pnh.param("coverage_interval_distance", ccpp_cfg.coverage_interval_distance, ccpp_cfg.robot_radius * 2 * 2);

        pnh.param("error_distance", ccpp_cfg.error_distance, 3);
        pnh.param("ccpp_map_origin_x", ccpp_cfg.ccpp_map_origin_x, 0);
        pnh.param("ccpp_map_origin_y", ccpp_cfg.ccpp_map_origin_y, 0);
        pnh.param("coverage_display", ccpp_cfg.coverage_display, false);
        pnh.param("path_eps", ccpp_cfg.path_eps, 1.0);
        pnh.param("inflation_distance", ccpp_cfg.inflation_distance, ccpp_cfg.robot_radius + 0.05);

        ccpp_cfg.predict_times = ccpp_cfg.coverage_interval_distance / ccpp_cfg.coverage_resolution * 1;

        ccpp_ptr_ = std::make_shared<ccpp_planner::CCPP>();
        ccpp_ptr_->configCCPP(ccpp_cfg);
    }

    bool CompleteCoveragePlanner::makePlan(const nav_msgs::OccupancyGrid& map,
                                           const nav_msgs::Path& edge_path,
                                           nav_msgs::Path& coverage_path,
                                           nav_msgs::Path& wall_path)
    {
        std::cout << "makePlan size is " << map.data.size() << std::endl;
        map_ = map;

        auto map_image = mapToImage(map);

        auto edge = pathToMapPose(edge_path);
        std::vector<ccpp_planner::MapPose> output_coverage;
        std::vector<ccpp_planner::MapPose> output_wall;

        if ( ccpp_ptr_->runCCPP(map_image, edge, output_coverage, output_wall) )
        {
            coverage_path = mapPoseToPath(output_coverage);
            wall_path = mapPoseToPath(output_wall);
            return true;
        }
        else
        {
            ROS_WARN("failed to find coverage path!");
            return false;
        }
    }

    cv::Mat CompleteCoveragePlanner::mapToImage(const nav_msgs::OccupancyGrid& map)
    {
        grid_map::GridMap tmp_grid_map;
        nav_msgs::OccupancyGrid tmp_map = map;
        for(auto it = tmp_map.data.begin() ; it != tmp_map.data.end(); it++)
        {
            if (*it < 0)
                *it = 0;
            else
                *it = abs(*it - 100);
        }
        ROS_WARN("mapToImage map.data[0] is %d, tmp_map.data[0] %d", map.data[0], tmp_map.data[0]);
        grid_map::GridMapRosConverter::fromOccupancyGrid(tmp_map, "coverage", tmp_grid_map);
        cv::Mat map_image;
        grid_map::GridMapCvConverter::toImage<unsigned char, 1>(tmp_grid_map, "coverage", CV_8UC1, 100, 0, map_image );

        map_image = map_image.t(); 
        cv::flip(map_image, map_image, 1);

        return map_image;
    }

    std::vector<ccpp_planner::MapPose> CompleteCoveragePlanner::pathToMapPose(const nav_msgs::Path& path)
    {
        std::vector<ccpp_planner::MapPose> map_poses;
        /*
        ccpp_planner::MapPose p1,p2,p3,p4;
        p1.x = 15;
        p1.y = 15;
        p2.x = 485;
        p2.y = 15;
        p3.x = 15;
        p3.y = 485;
        p4.x = 485;
        p4.y = 485;

        map_poses.push_back(p1);
        map_poses.push_back(p2);
        map_poses.push_back(p4);
        map_poses.push_back(p3);
        */

        ccpp_planner::MapPose pt;
        for (auto it = path.poses.begin(); it != path.poses.end(); ++it)
        {
            ROS_WARN("pose x is %f, pose y is %f", it->pose.position.x,it->pose.position.y);
            pt.x = (int)((it->pose.position.x - map_.info.origin.position.x)/map_.info.resolution);
            pt.y = (int)(map_.info.height - (it->pose.position.y - map_.info.origin.position.y)/map_.info.resolution);

            // pt.x = (int)((it->pose.position.x - map_.info.origin.position.x) / 0.05);
            // pt.y = (int)((it->pose.position.y - map_.info.origin.position.y) / 0.05);
            map_poses.push_back(pt);
            ROS_WARN("pt.x is %d, pt.y is %d", pt.x, pt.y);
        }

        return map_poses;
    }

   nav_msgs::Path CompleteCoveragePlanner::mapPoseToPath(const std::vector<ccpp_planner::MapPose>& poses)
   {
        nav_msgs::Path path;
        geometry_msgs::PoseStamped path_pose;
        path_pose.header.frame_id = "map";

        for (auto p = poses.begin(); p != poses.end(); p++)
        {
            path_pose.pose.position.x = p->x * map_.info.resolution + map_.info.origin.position.x;
            path_pose.pose.position.y = (map_.info.height - p->y) * map_.info.resolution + map_.info.origin.position.y;
            path.poses.push_back(path_pose);
        }

        path.header.frame_id = "map";

        return path;
   }

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(complete_coverage_planner::CompleteCoveragePlanner, navit_core::CoveragePlanner)
