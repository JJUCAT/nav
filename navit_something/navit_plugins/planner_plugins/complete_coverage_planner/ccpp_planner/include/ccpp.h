//
// Created by forrest on 18-2-28.
//

#ifndef CCPP_H
#define CCPP_H

#include <iostream>
#include <vector>
#include <math.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "astar_gridmap_2d.h"
#include "ccpp_type.h"
#include "room_rotator.h"

/**
 * 坐标系: ------> x
 *        |
 *        v
 *        y
 */

namespace ccpp_planner
{
    struct CcppConfig
    {
      CoverageDirection coverage_direction = CoverageDirection::HORIZONTAL;       // coverage direction
      CoverageStartDirection coverage_start_direction = CoverageStartDirection::LEFTUP;    // coverage start direction

      double map_resolution = 0.05;              // [m]
      double coverage_resolution = 0.1;         // [m]
      double robot_radius = 0.2;                // [m]
      double coverage_interval_distance = robot_radius * 2 * 2;  // [m]
      double inflation_distance = robot_radius + 0.05;          // [m]
      double path_eps = 1.0;        // [pixel],for downsample path,the distance between points when generating a path
      int predict_times = coverage_interval_distance / coverage_resolution * 1;      // [pixel],predict distance on map
      int error_distance = 3;     // [pixel],error distance for turn circle ccpp
      int ccpp_map_origin_x = 0;  // [pixel],ccpp map origin x value on map
      int ccpp_map_origin_y = 0;  // [pixel],ccpp map origin y value on map

      bool coverage_display = false; // visualize the coverage process
    };
   
    class CCPP
    {
    public:

        CCPP();
        ~CCPP();

        bool runCCPP(const cv::Mat& map,
                     const std::vector<MapPose>& edge_path,
                     std::vector<MapPose>& output_ccpp_path,
                     std::vector<MapPose>& output_wall_path);

        void configCCPP(CcppConfig config);

    private:

        bool initCCPP(const cv::Mat& map,
                      const std::vector<MapPose>& edge_path,
                      cv::Mat& ccpp_map,
                      cv::Mat& inflation_map);

        void getCoveragePath(const cv::Mat& ccpp_map,
                             const std::vector<MapPose>& downsampled_path,
                             std::vector<MapPose>& output_ccpp_path);

        void coverageDisplay(const cv::Mat& original_map,
                             const std::vector<MapPose>& output_ccpp_path);

        void wallPoseDisplay(const cv::Mat& original_map,
                             const std::vector<std::vector<MapPose>>& wall_pose_list);

        void getMoveRightPathWithoutObstacle(const std::vector<MapPose>& x_wall_pose_list);
        void getMoveLeftPathWithoutObstacle(const std::vector<MapPose>& x_wall_pose_list);

        bool getMoveRightPathWithObstacle(const cv::Mat& inflation_map,
                                          const std::vector<MapPose>& x_wall_pose_list);
        bool getMoveLeftPathWithObstacle(const cv::Mat& inflation_map,
                                         const std::vector<MapPose>& x_wall_pose_list);

        bool getRightDownPath(const cv::Mat& inflation_map,
                              const std::vector<MapPose>& next_x_wall_pose_list);
        bool getLeftDownPath(const cv::Mat& inflation_map,
                             const std::vector<MapPose>& next_x_wall_pose_list);

        void getCoverageWallPose(const cv::Mat& ccpp_map,
                                 const MapPose& start_pose,
                                 std::vector<std::vector<MapPose>>& wall_pose_list,
                                 std::vector<std::vector<MapPose>>& turn_wall_pose_lis);

        void getContourWallPose(const cv::Mat& ccpp_map,
                                const MapPose& start_pose,
                                std::vector<std::vector<MapPose>>& wall_pose_list);

        CoverageDirection getCoverageDirection(const cv::Mat& ccpp_map);

        void getCoverageMap(const cv::Mat& in_map,
                            const std::vector<MapPose>& edge_path,
                            cv::Mat& out_map);

        bool findStartPose(const cv::Mat& ccpp_map, MapPose& start_pose);

        // downsamples a given path original_path to waypoint distances
        // of path_eps and appends the resulting path to downsampled_path
        void downsamplePath(const std::vector<MapPose>& original_path,
                            std::vector<MapPose>& downsampled_path,
                            MapPose& original_path_start_pose,
                            const double path_eps);

        MapPose start_pose;                         // coverage start pose
        MapPose current_pose;
        MapPose next_pose;
        std::vector<MapPose> ccpp_path;             // coverage output path
        std::vector<MapPose> downsample_ccpp_path;  // downsample coverage output path
        std::vector<MapPose> raw_wall_path;  
        CoverageDirection coverage_direction;       // coverage direction
        CoverageStartDirection coverage_start_direction;    // coverage start direction

        AStar::PathFinder generator_;

        double map_resolution;              // [m]
        double coverage_resolution;         // [m]
        double robot_radius;                // [m]
        double coverage_interval_distance;  // [m]
        double inflation_distance;          // [m]
        double path_eps;        // [pixel],for downsample path,the distance between points when generating a path
        int predict_times;      // [pixel],predict distance on map
        int error_distance;     // [pixel],error distance for turn circle ccpp
        int ccpp_map_origin_x;  // [pixel],ccpp map origin x value on map
        int ccpp_map_origin_y;  // [pixel],ccpp map origin y value on map
        RoomRotator room_rotation;  // rotation class
        cv::Mat R_;                 // rotation matrix

        CcppConfig config_;
    };
}

#endif //CCPP_H
