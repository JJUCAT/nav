//
// Created by fan on 23-8-7.
//

#include "ipa_boustrophedon_explorator/boustrophedon_explorator_ros.h"
#include <opencv4/opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ipa_boustrophedon_explorator::BoustrophedonExploratorRos, navit_core::CoveragePlanner)

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

namespace ipa_boustrophedon_explorator
{
void BoustrophedonExploratorRos::initialize(const std::string& name,
                                            const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros)
{
  name_ = name;
  costmap_ros_ = costmap_ros;
  node_handle_ = ros::NodeHandle("~/" + name);

  // setup dynamic reconfigure
  dynamic_reconfigure_.setCallback(boost::bind(&BoustrophedonExploratorRos::reconfigureCB, this, _1, _2));

  node_handle_.param("boustrophedon_exploration_algorithm", config_.boustrophedon_exploration_algorithm, 2);
  if (config_.boustrophedon_exploration_algorithm != 2 || config_.boustrophedon_exploration_algorithm != 8)
  {
    config_.boustrophedon_exploration_algorithm = 2;
  }
  std::cout << "room_exploration/boustrophedon_exploration_algorithm = " << config_.boustrophedon_exploration_algorithm
            << std::endl;

  node_handle_.param("robot_radius", config_.robot_radius, 0.3);
  std::cout << "room_exploration/robot_radius = " << config_.robot_radius << std::endl;

  node_handle_.param("coverage_radius", config_.coverage_radius, 0.3);
  std::cout << "room_exploration/coverage_radius = " << config_.coverage_radius << std::endl;

  node_handle_.param("map_correction_closing_neighborhood_size", config_.map_correction_closing_neighborhood_size, 3);
  std::cout << "room_exploration/map_correction_closing_neighborhood_size = "
            << config_.map_correction_closing_neighborhood_size << std::endl;

  node_handle_.param("min_cell_area", config_.min_cell_area, 10.0);
  std::cout << "room_exploration/min_cell_area = " << config_.min_cell_area << std::endl;

  node_handle_.param("path_eps", config_.path_eps, 2.0);
  std::cout << "room_exploration/path_eps = " << config_.path_eps << std::endl;

  node_handle_.param("grid_obstacle_offset", config_.grid_obstacle_offset, 0.0);
  std::cout << "room_exploration/grid_obstacle_offset = " << config_.grid_obstacle_offset << std::endl;

  node_handle_.param("max_deviation_from_track", config_.max_deviation_from_track, -1);
  std::cout << "room_exploration/max_deviation_from_track = " << config_.max_deviation_from_track << std::endl;

  node_handle_.param("cell_visiting_order", config_.cell_visiting_order, 1);
  std::cout << "room_exploration/cell_visiting_order = " << config_.cell_visiting_order << std::endl;
}

bool BoustrophedonExploratorRos::makePlan(const nav_msgs::Path& edge_path, const geometry_msgs::Pose& start_pose,
                                          nav_msgs::Path& coverage_path)
{
  auto costmap = costmap_ros_->getCostmap();

  ROS_INFO("***** BoustrophedonExploratorRos::makePlan *****");

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

  // get the grid size, to check the areas that should be revisited later
  // is the square grid cell side length that fits into the circle with the
  // robot's coverage radius or fov coverage radius
  double grid_spacing_in_meter = config_.coverage_radius * std::sqrt(2);

  // map the grid size to an int in pixel coordinates, using floor method
  // is the square grid cell side length that fits into the circle with the robot's coverage radius
  // or fov coverage radius, multiply with sqrt(2) to receive the whole working width
  const double grid_spacing_in_pixel = grid_spacing_in_meter / map_resolution;
  std::cout << "grid size: " << grid_spacing_in_meter << " m   (" << grid_spacing_in_pixel << " px)" << std::endl;

  // ***************** II. plan the path using the wanted planner *****************
  // todo: consider option to provide the inflated map or the robot radius to the functions instead of inflating with
  // half cell size there
  Eigen::Matrix<float, 2, 1> zero_vector;
  zero_vector << 0, 0;
  std::vector<geometry_msgs::Pose2D> exploration_path;

  if (config_.boustrophedon_exploration_algorithm == 2)
  {
    boustrophedon_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position,
                                               map_origin, grid_spacing_in_pixel, config_.grid_obstacle_offset,
                                               config_.path_eps, config_.cell_visiting_order, true, zero_vector,
                                               config_.min_cell_area, config_.max_deviation_from_track);
  }
  else if (config_.boustrophedon_exploration_algorithm == 8)
  {
    boustrophedon_variant_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position,
                                                       map_origin, grid_spacing_in_pixel, config_.grid_obstacle_offset,
                                                       config_.path_eps, config_.cell_visiting_order, true, zero_vector,
                                                       config_.min_cell_area, config_.max_deviation_from_track);
  }
  else
  {
    return false;
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

void BoustrophedonExploratorRos::reconfigureCB(BoustrophedonExploratorConfig& config, uint32_t level)
{
  std::cout << "reconfigureCB..." << std::endl;

  config_.boustrophedon_exploration_algorithm = config.boustrophedon_exploration_algorithm;
  if (config_.boustrophedon_exploration_algorithm != 2 || config_.boustrophedon_exploration_algorithm != 8)
  {
    config_.boustrophedon_exploration_algorithm = 2;
  }
  std::cout << "room_exploration/boustrophedon_exploration_algorithm = " << config_.boustrophedon_exploration_algorithm
            << std::endl;

  config_.robot_radius = config.robot_radius;
  std::cout << "room_exploration/robot_radius = " << config_.robot_radius << std::endl;

  config_.coverage_radius = config.coverage_radius;
  std::cout << "room_exploration/coverage_radius = " << config_.coverage_radius << std::endl;

  config_.map_correction_closing_neighborhood_size = config.map_correction_closing_neighborhood_size;
  std::cout << "room_exploration/map_correction_closing_neighborhood_size = "
            << config_.map_correction_closing_neighborhood_size << std::endl;

  config_.min_cell_area = config.min_cell_area;
  std::cout << "room_exploration/min_cell_are_ = " << config_.min_cell_area << std::endl;

  config_.path_eps = config.path_eps;
  std::cout << "room_exploration/path_eps = " << config_.path_eps << std::endl;

  config_.grid_obstacle_offset = config.grid_obstacle_offset;
  std::cout << "room_exploration/grid_obstacle_offset = " << config_.grid_obstacle_offset << std::endl;

  config_.max_deviation_from_track = config.max_deviation_from_track;
  std::cout << "room_exploration/max_deviation_from_track = " << config_.max_deviation_from_track << std::endl;

  config_.cell_visiting_order = config.cell_visiting_order;
  std::cout << "room_exploration/cell_visiting_order = " << config_.cell_visiting_order << std::endl;
}

}  // namespace ipa_boustrophedon_explorator
