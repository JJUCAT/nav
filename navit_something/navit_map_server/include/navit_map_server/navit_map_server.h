//
// Created by fan on 23-8-14.
//

#ifndef NAVIT_MAP_SERVER_NAVIT_MAP_SERVER_H
#define NAVIT_MAP_SERVER_NAVIT_MAP_SERVER_H
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include "navit_map_server/message_navit_map.pb.h"
#include <navit_msgs/GetMapStationPose.h>
#include <navit_msgs/GetMapRoutePath.h>
#include <navit_msgs/GetMapAreaEdge.h>
#include <navit_msgs/BuildGridMap.h>
#include <navit_msgs/SwitchTaskMap.h>
namespace navit_map_server
{

class NavitMapServer
{
public:
  /// pub
  inline const static char* TOPIC_MAP_INFO = "/navit/map_info";
  inline const static char* TOPIC_MAP_INFO_DISPLAY = "/navit/map_info_display";
  inline const static char* TOPIC_MAP_FORBIDDEN_LINE = "/navit/map_forbidden_line";
  inline const static char* TOPIC_MAP_FORBIDDEN_AREA = "/navit/map_forbidden_area";

  /// sub
  inline const static char* TOPIC_MAP_INFO_UPDATE = "/navit/map_info_update";
  inline const static char* TOPIC_SWITCH_MAP = "/navit/switch_map";

  /// service
  inline const static char* SERVICE_MAP_INFO_SAVE = "/navit/map_info_save";

  /// service for bt nodes
  inline const static char* SERVICE_GET_MAP_STATION_POSE = "/get_map_station_pose";
  inline const static char* SERVICE_GET_MAP_ROUTE_PATH = "/get_map_route_path";
  inline const static char* SERVICE_GET_MAP_AREA_EDGE = "/get_map_area_edge";

  using Point = navit::protocol::map_info::Point;
  using MapPoint = navit::protocol::map_info::MapPoint;
  using MapForbiddenLine = navit::protocol::map_info::MapLine;
  using MapCurve = navit::protocol::map_info::MapLine;
  using MapArea = navit::protocol::map_info::MapArea;

  using TotalMapInfo = navit::protocol::map_info::TotalMapInfo;
  using MapInfo = navit::protocol::map_info::MapInfo;

  explicit NavitMapServer(const std::string& map_file);

private:
  void update();
  void update_publish() const;
  void update_rviz_display();

private:
  void switchMap(int map_index);
  void onSwitchMap(const std_msgs::String::ConstPtr& msg);
  void onMapInfoUpdate(const std_msgs::String::ConstPtr& msg);
  bool saveMapInfoCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool getMapStationPoseCallback(navit_msgs::GetMapStationPose::Request& req,
                                 navit_msgs::GetMapStationPose::Response& res);
  bool getMapRoutePathCallback(navit_msgs::GetMapRoutePath::Request& req, navit_msgs::GetMapRoutePath::Response& res);
  bool getMapAreaEdgeCallback(navit_msgs::GetMapAreaEdge::Request& req, navit_msgs::GetMapAreaEdge::Response& res);
  static geometry_msgs::PoseStamped convertToPoseStamped(const Point& point);

private:
  void clear_markers();
  void draw_station(visualization_msgs::MarkerArray& markerArray, const MapPoint& station, int& id,
                    visualization_msgs::Marker& base_marker);
  void draw_forbidden_line(visualization_msgs::MarkerArray& markerArray, const MapForbiddenLine& line, int& id,
                           visualization_msgs::Marker& base_marker);
  void draw_curve(visualization_msgs::MarkerArray& markerArray, const MapCurve& curve, int& id,
                  visualization_msgs::Marker& base_marker);
  void draw_polygon(visualization_msgs::MarkerArray& markerArray, const MapArea& area, int& id,
                    visualization_msgs::Marker& base_marker);

private:
  const MapPoint* get_station(const std::string& station_name);

  const MapPoint* get_station(const int64_t& station_id);

  std::string get_station_name(const int64_t& station_id);

  const MapCurve* get_curve(const std::string& start, const std::string& end);

  bool generate_route_path(const std::vector<std::string>& via_point_order, double distance_interval,
                           nav_msgs::Path& path);

  double generate_curve_path(const MapCurve* curve, double distance_interval, nav_msgs::Path* out_path);

  /**
   * 生成贝塞尔路径
   * @param name
   * @param distance_interval
   * @param out_path if nullptr only return distance
   * @return 曲线长度 单位m  <=0生成失败
   */
  double generate_bezier_path(const MapCurve* curve, double distance_interval, nav_msgs::Path* out_path);

  /**
   * 生成圆弧路径
   * @param name
   * @param distance_interval
   * @param out_path if nullptr only return distance
   * @return 曲线长度 单位m  <=0生成失败
   */
  double generate_arc_path(const MapCurve* curve, double distance_interval, nav_msgs::Path* out_path);

  /**
   * 生成直线路径
   * @param name
   * @param distance_interval
   * @param out_path if nullptr only return distance
   * @return 曲线长度 单位m  <=0生成失败
   */
  double generate_straight_path(const MapCurve* curve, double distance_interval, nav_msgs::Path* out_path);
  
  //add by czk
  /**
   * @brief 多边形转栅格server 
   * 
   * @param req 
   * @param res 
   * @return true 
   * @return false 
   */
  bool rasterizePolygonCallback(navit_msgs::BuildGridMap::Request &req,
                                navit_msgs::BuildGridMap::Response &res);

  bool saveMapMetadata(const std::string& mapDirectory, const std::string& yamlFilename, 
                       const std::string& imageFilename, float resolution, 
                       const cv::Point& origin, const cv::Size& imageSize);

  bool switchMapCallback(navit_msgs::SwitchTaskMap::Request &req,
                         navit_msgs::SwitchTaskMap::Response &res);

  bool loadMapMetadata(const std::string& yaml_file_path, nav_msgs::OccupancyGrid& grid_msg);

  bool switchMapFile(const std::string& map_dir, const std::string& map_name);
private:
  ros::NodeHandle nh_{"~"};

  /// topic
  ros::Subscriber map_info_update_sub_;
  ros::Subscriber switch_map_sub_;

  ros::Publisher map_info_pub_;
  ros::Publisher map_info_display_pub_;
  ros::Publisher map_forbidden_line_pub_;
  ros::Publisher map_forbidden_area_pub_;

  /// service
  ros::ServiceServer map_info_save_server_;
  ros::ServiceServer get_station_pose_server_;
  ros::ServiceServer get_route_path_server_;
  ros::ServiceServer get_area_edge_server_;

  ros::ServiceServer rasterize_polygon_server_;
  ros::ServiceServer switch_map_server_;

  ros::Publisher map_pub_;
private:
  bool map_ok_{};
  std::string map_file_;
  int current_map_info_index_{};
  MapInfo current_map_info_proto_;
  std::string current_map_info_string_;
  std::map<int64_t, std::string> current_map_station_id_name_map_;

  TotalMapInfo total_map_info_proto_;
  std::string total_map_info_string_;

  std::string map_dir_ = "/home/robot/navit_map";
};

}  // namespace navit_map_server

#endif  // NAVIT_MAP_SERVER_NAVIT_MAP_SERVER_H
