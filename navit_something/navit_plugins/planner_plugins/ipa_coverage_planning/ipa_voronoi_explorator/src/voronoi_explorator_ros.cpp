//
// Created by fan on 23-8-7.
//

#include "ipa_voronoi_explorator/voronoi_explorator_ros.h"
#include <opencv4/opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ipa_room_exploration/voronoi.hpp"
#include "ipa_room_exploration/room_rotator.h"

#include "pluginlib/class_list_macros.hpp"
#include "ipa_building_navigation/A_star_pathplanner.h"

PLUGINLIB_EXPORT_CLASS(ipa_voronoi_explorator::VoronoiExploratorRos, navit_core::CoveragePlanner)

static bool hasBlackPixels(const cv::Mat& image, int x1, int y1, int x2, int y2) {
    // 确定直线方向和距离
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    // 逐个遍历像素点
    while (true) {
        // 检查当前像素点是否为黑色
        if(image.at<uchar>(y1, x1) == 0){
            return true;
        }

        // 结束条件：到达终点
        if (x1 == x2 && y1 == y2) {
            break;
        }

        // 计算下一个像素点
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }

    return false;
}


// function to create an occupancyGrid map out of a given cv::Mat
static void matToMap(nav_msgs::OccupancyGrid& map, const cv::Mat& mat)
{
  map.info.width = mat.cols;
  map.info.height = mat.rows;
  map.data.resize(mat.cols * mat.rows);

  for (int x = 0; x < mat.cols; x++)
    for (int y = 0; y < mat.rows; y++)
      map.data[y * mat.cols + x] = mat.at<int8_t>(y, x) ? 0 : 100;
}

// function to create a cv::Mat out of a given occupancyGrid map
static void mapToMat(const nav_msgs::OccupancyGrid& map, cv::Mat& mat)
{
  mat = cv::Mat(map.info.height, map.info.width, CV_8U);

  for (int x = 0; x < mat.cols; x++)
    for (int y = 0; y < mat.rows; y++)
      mat.at<int8_t>(y, x) = map.data[y * mat.cols + x];
}

static void costmapToMat(const navit_costmap::Costmap2D* costmap, cv::Mat& mat)
{
  static const unsigned char NO_INFORMATION = 255;
  static const unsigned char LETHAL_OBSTACLE = 254;
  static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
  static const unsigned char FREE_SPACE = 0;

  // 获取Costmap的尺寸信息
  int width = costmap->getSizeInCellsX();
  int height = costmap->getSizeInCellsY();

  // 创建一个黑色的图像，尺寸与Costmap相同
  mat = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));

  // 将Costmap数据转换为图像数据
  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      unsigned char value = costmap->getCost(j, i);

      if (value == LETHAL_OBSTACLE || value == INSCRIBED_INFLATED_OBSTACLE)
      {
        mat.at<uchar>(i, j) = 0;  // 设置障碍物为黑色
      }
      else if (value == FREE_SPACE)
      {
        mat.at<uchar>(i, j) = 255;  // 设置自由空间为白色
      }
      else
      {
        mat.at<uchar>(i, j) = 127;  // 设置未知空间为灰色
      }
    }
  }
}

static bool removeUnconnectedRoomParts(cv::Mat& room_map)
{
  // create new map with segments labeled by increasing labels from 1,2,3,...
  cv::Mat room_map_int(room_map.rows, room_map.cols, CV_32SC1);
  for (int v = 0; v < room_map.rows; ++v)
  {
    for (int u = 0; u < room_map.cols; ++u)
    {
      if (room_map.at<uchar>(v, u) == 255)
        room_map_int.at<int32_t>(v, u) = -100;
      else
        room_map_int.at<int32_t>(v, u) = 0;
    }
  }

  std::map<int, int> area_to_label_map;  // maps area=number of segment pixels (keys) to the respective label (value)
  int label = 1;
  for (int v = 0; v < room_map_int.rows; ++v)
  {
    for (int u = 0; u < room_map_int.cols; ++u)
    {
      if (room_map_int.at<int32_t>(v, u) == -100)
      {
        const int area =
            cv::floodFill(room_map_int, cv::Point(u, v), cv::Scalar(label), 0, 0, 0, 8 | cv::FLOODFILL_FIXED_RANGE);
        area_to_label_map[area] = label;
        ++label;
      }
    }
  }
  // abort if area_to_label_map.size() is empty
  if (area_to_label_map.size() == 0)
    return false;

  // remove all room pixels from room_map which are not accessible
  const int label_of_biggest_room = area_to_label_map.rbegin()->second;
  std::cout << "label_of_biggest_room=" << label_of_biggest_room << std::endl;
  for (int v = 0; v < room_map.rows; ++v)
    for (int u = 0; u < room_map.cols; ++u)
      if (room_map_int.at<int32_t>(v, u) != label_of_biggest_room)
        room_map.at<uchar>(v, u) = 0;

  return true;
}

namespace ipa_voronoi_explorator
{
void VoronoiExploratorRos::initialize(const std::string& name,
                                      const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros)
{
  name_ = name;
  costmap_ros_ = costmap_ros;
  node_handle_ = ros::NodeHandle("~/" + name);

  // setup dynamic reconfigure
  dynamic_reconfigure_.setCallback(boost::bind(&VoronoiExploratorRos::reconfigureCB, this, _1, _2));

  node_handle_.param("robot_radius", config_.robot_radius, 0.3);
  std::cout << "room_exploration/robot_radius = " << config_.robot_radius << std::endl;

  node_handle_.param("coverage_radius", config_.coverage_radius, 0.3);
  std::cout << "room_exploration/coverage_radius = " << config_.coverage_radius << std::endl;

  node_handle_.param("map_correction_closing_neighborhood_size", config_.map_correction_closing_neighborhood_size, 3);
  std::cout << "room_exploration/map_correction_closing_neighborhood_size = "
            << config_.map_correction_closing_neighborhood_size << std::endl;
}

bool VoronoiExploratorRos::makePlan(const nav_msgs::Path& edge_path, const geometry_msgs::Pose& start_pose,
                                    nav_msgs::Path& coverage_path)
{
  auto costmap = costmap_ros_->getCostmap();

  ROS_INFO("***** VoronoiExploratorRos::makePlan *****");

  // ***************** I. read the given parameters out of the goal *****************
  // todo: this is only correct if the map is not rotated
  const cv::Point2d map_origin(costmap->getOriginX(), costmap->getOriginY());
  const float map_resolution = costmap->getResolution();  // in [m/cell]
  std::cout << "map origin: " << map_origin << " m       map resolution: " << map_resolution << " m/cell" << std::endl;

  const float robot_radius = config_.robot_radius;
  const int robot_radius_in_pixel = robot_radius / map_resolution;
  std::cout << "robot radius: " << robot_radius << " m   (" << robot_radius_in_pixel << " px)" << std::endl;

  const cv::Point starting_position((start_pose.position.x - map_origin.x) / map_resolution,
                                    (start_pose.position.y - map_origin.y) / map_resolution);
  std::cout << "starting point: (" << start_pose.position.x << ", " << start_pose.position.y << ") m   ("
            << starting_position << " px)" << std::endl;

  cv::Mat room_map;
  costmapToMat(costmap, room_map);

  // 区域内规划
  if (edge_path.poses.size() >= 3)
  {
    std::vector<cv::Point> points;
    points.reserve(edge_path.poses.size());
    for (const auto& pose : edge_path.poses)
    {
      unsigned int mx, my;
      costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
      points.emplace_back(mx, my);
    }

    cv::Mat img_mask(room_map.rows, room_map.cols, CV_8UC1, cv::Scalar(255));
    cv::line(img_mask, points.front(), points.back(), cv::Scalar(0), 2, 8, 0);
    for (auto it = points.begin(); it != points.end() - 1; ++it)
    {
      cv::line(img_mask, *it, *(it + 1), cv::Scalar(0), 2, 8, 0);
    }

    cv::floodFill(img_mask, cv::Point(0, 0), cv::Scalar(0));
    room_map.setTo(0, img_mask == 0);
  }

  // determine room size
  int area_px = 0;  // room area in pixels
  for (int v = 0; v < room_map.rows; ++v)
    for (int u = 0; u < room_map.cols; ++u)
      if (room_map.at<uchar>(v, u) >= 250)
        area_px++;
  std::cout << "### room area = " << area_px * map_resolution * map_resolution << " m^2" << std::endl;

  // closing operation to neglect inaccessible areas and map errors/artifacts
  cv::Mat temp;
  cv::erode(room_map, temp, cv::Mat(), cv::Point(-1, -1), config_.map_correction_closing_neighborhood_size);
  cv::dilate(temp, room_map, cv::Mat(), cv::Point(-1, -1), config_.map_correction_closing_neighborhood_size);

  // remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with
  // the largest area
  const bool room_not_empty = removeUnconnectedRoomParts(room_map);
  if (!room_not_empty)
  {
    std::cout << "RoomExplorationServer::exploreRoom: Warning: the requested room is too small for generating "
                 "exploration trajectories."
              << std::endl;
    return false;
  }

  nav_msgs::OccupancyGrid room_gridmap;
  matToMap(room_gridmap, room_map);

  // get the grid size, to check the areas that should be revisited later
  // is the square grid cell side length that fits into the circle with the
  // robot's coverage radius or fov coverage radius
  double grid_spacing_in_meter = config_.coverage_radius * std::sqrt(2);

  // map the grid size to an int in pixel coordinates, using floor method
  // is the square grid cell side length that fits into the circle with the robot's coverage radius
  // or fov coverage radius, multiply with sqrt(2) to receive the whole working width
  const double grid_spacing_in_pixel = grid_spacing_in_meter / map_resolution;
  std::cout << "grid size: " << grid_spacing_in_meter << " m   (" << grid_spacing_in_pixel << " px)" << std::endl;

  const int grid_spacing_as_int = (int)std::floor(grid_spacing_in_pixel);
  std::cout << "grid spacing in pixel: " << grid_spacing_as_int << std::endl;

  // create the object that plans the path, based on the room-map
  VoronoiMap vm(room_gridmap.data.data(), room_gridmap.info.width, room_gridmap.info.height, grid_spacing_as_int, 2,
                true);  // coverage_diameter-1); // diameter in pixel (full working width can be used here because
                        // tracks are planned in parallel motion)
  // get the exploration path
  std::vector<geometry_msgs::Pose2D> exploration_path_uncleaned;
  vm.setSingleRoom(true);                // to force to consider all rooms
  vm.generatePath(exploration_path_uncleaned, cv::Mat(), starting_position.x,
                  starting_position.y);  // start position in room center

  //  cv::imshow("room_map", room_map);
  //  cv::waitKey(1000);
  //  cv::Mat room_map2 = room_map.clone();
  AStarPlanner path_planner;
  std::vector<geometry_msgs::Pose2D> temp_exploration_path;
  temp_exploration_path.push_back(exploration_path_uncleaned.front());
  for(int i=0; i<exploration_path_uncleaned.size() - 1; ++i){
      auto start = exploration_path_uncleaned[i];
      auto end = exploration_path_uncleaned[i+1];
      if(hasBlackPixels(room_map, start.x, start.y, end.x, end.y)){
          cv::Point p1(start.x, start.y);
          cv::Point p2(end.x, end.y);
          // cv::line(room_map2, p1, p2, cv::Scalar(0, 0, 255), 2, 8, 0);

          std::vector<cv::Point> astar_path_pass_though_obs; // 跨越障碍物
          path_planner.planPath(room_map, p1, p2, 1.0, 0, map_resolution, 0, &astar_path_pass_though_obs);

          if(astar_path_pass_though_obs.size() < 2){
              temp_exploration_path.push_back(end);
              std::cout << "path_planner find path pass though obs failed! \n"
                        << "start: (" << start.x << ", " << start.y << ") end: (" << end.x << ", " << end.y << ")" << std::endl;
              continue;
          }
          for(int k=1; k < astar_path_pass_though_obs.size() - 1; ++k){
              geometry_msgs::Pose2D pose;
              pose.x = astar_path_pass_though_obs[k].x;
              pose.y = astar_path_pass_though_obs[k].y;
              temp_exploration_path.push_back(pose);
          }
      }
      else{
          temp_exploration_path.push_back(end);
      }
  }
  temp_exploration_path.push_back(exploration_path_uncleaned.back());
  // cv::imshow("room_map2", room_map2);
  // cv::waitKey(1000);

  // clean path from subsequent double occurrences of the same pose
  std::vector<geometry_msgs::Pose2D> exploration_path;
  downsampleTrajectory(temp_exploration_path, exploration_path, 3.5 * 3.5);  // 3.5*3.5);


  // convert to poses with angles
  RoomRotator room_rotation;
  room_rotation.transformPointPathToPosePath(exploration_path);

  // transform to global coordinates
  for (size_t pos = 0; pos < exploration_path.size(); ++pos)
  {
    exploration_path[pos].x = (exploration_path[pos].x * map_resolution) + map_origin.x;
    exploration_path[pos].y = (exploration_path[pos].y * map_resolution) + map_origin.y;
  }

  coverage_path.header.frame_id = "map";
  coverage_path.header.stamp = ros::Time::now();
  for (auto& p : exploration_path)
  {
    // 创建一个 geometry_msgs::PoseStamped 对象
    geometry_msgs::PoseStamped pose_stamped;

    // 设置该位姿点的时间戳
    pose_stamped.header.stamp = coverage_path.header.stamp;

    // 设置该位姿点的坐标系
    pose_stamped.header.frame_id = coverage_path.header.frame_id;

    // 将 exploration_path 中的位姿信息赋值给 pose_stamped
    pose_stamped.pose.position.x = p.x;
    pose_stamped.pose.position.y = p.y;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, p.theta);
    pose_stamped.pose.orientation = tf2::toMsg(quat);

    // 将 pose_stamped 添加到 coverage_path.poses 中
    coverage_path.poses.push_back(pose_stamped);
  }

  return true;
}

void VoronoiExploratorRos::downsampleTrajectory(const std::vector<geometry_msgs::Pose2D>& path_uncleaned,
                                                std::vector<geometry_msgs::Pose2D>& path, const double min_dist_squared)
{
  // clean path from subsequent double occurrences of the same pose
  path.push_back(path_uncleaned[0]);
  cv::Point last_added_point(path_uncleaned[0].x, path_uncleaned[0].y);
  for (size_t i = 1; i < path_uncleaned.size(); ++i)
  {
    const cv::Point current_point(path_uncleaned[i].x, path_uncleaned[i].y);
    cv::Point vector = current_point - last_added_point;
    if (vector.x * vector.x + vector.y * vector.y > min_dist_squared || i == path_uncleaned.size() - 1)
    {
      path.push_back(path_uncleaned[i]);
      last_added_point = current_point;
    }
  }
}

void VoronoiExploratorRos::reconfigureCB(DynamicConfig& config, uint32_t level)
{
  std::cout << "reconfigureCB..." << std::endl;

  config_.robot_radius = config.robot_radius;
  std::cout << "room_exploration/robot_radius = " << config_.robot_radius << std::endl;

  config_.coverage_radius = config.coverage_radius;
  std::cout << "room_exploration/coverage_radius = " << config_.coverage_radius << std::endl;

  config_.map_correction_closing_neighborhood_size = config.map_correction_closing_neighborhood_size;
  std::cout << "room_exploration/map_correction_closing_neighborhood_size = "
            << config_.map_correction_closing_neighborhood_size << std::endl;
}

}  // namespace ipa_voronoi_explorator
