//
// Created by fan on 23-8-14.
//

#include "navit_map_server/navit_map_server.h"
#include <fstream>
#include <google/protobuf/util/json_util.h>
#include <tf2/LinearMath/Quaternion.h>
#include "bezier.hpp"
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Path.h>
#include <navit_msgs/PolygonArray.h>
#include <navit_msgs/LineArray.h>

static bool save_file(const std::string& file_name, const std::string& file_content)
{
  std::ofstream fout(file_name);
  if (!fout)
  {
    std::cerr << "Error opening file " << file_name << std::endl;
    return false;
  }
  fout << file_content;
  fout.close();
  return true;
}

static std::string read_file(const char* file_name)
{
  std::ifstream fin(file_name);
  if (!fin)
  {
    std::cerr << "Error opening file " << file_name << std::endl;
    return "";
  }
  std::string file_string((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
  fin.close();
  return file_string;
}

static geometry_msgs::Point32 convert(const navit_map_server::NavitMapServer::Point& map_point)
{
  geometry_msgs::Point32 point;
  point.x = map_point.x();
  point.y = map_point.y();
  point.z = map_point.z();
  return point;
}

namespace navit_map_server
{
NavitMapServer::NavitMapServer(const std::string& map_file)
{
  map_file_ = map_file;
  total_map_info_string_ = read_file(map_file_.c_str());
  nh_.param("map_dir", map_dir_, map_dir_);
  /// pub
  map_info_pub_ = nh_.advertise<std_msgs::String>(TOPIC_MAP_INFO, 1, true);
  map_info_display_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(TOPIC_MAP_INFO_DISPLAY, 1, true);
  map_forbidden_line_pub_ = nh_.advertise<navit_msgs::LineArray>(TOPIC_MAP_FORBIDDEN_LINE, 1, true);
  map_forbidden_area_pub_ = nh_.advertise<navit_msgs::PolygonArray>(TOPIC_MAP_FORBIDDEN_AREA, 1, true);

  /// sub
  map_info_update_sub_ = nh_.subscribe(TOPIC_MAP_INFO_UPDATE, 1, &NavitMapServer::onMapInfoUpdate, this);
  switch_map_sub_ = nh_.subscribe(TOPIC_SWITCH_MAP, 1, &NavitMapServer::onSwitchMap, this);

  /// service
  map_info_save_server_ = nh_.advertiseService(SERVICE_MAP_INFO_SAVE, &NavitMapServer::saveMapInfoCallback, this);

  /// service for bt nodes
  get_station_pose_server_ =
      nh_.advertiseService(SERVICE_GET_MAP_STATION_POSE, &NavitMapServer::getMapStationPoseCallback, this);
  get_route_path_server_ =
      nh_.advertiseService(SERVICE_GET_MAP_ROUTE_PATH, &NavitMapServer::getMapRoutePathCallback, this);
  get_area_edge_server_ =
      nh_.advertiseService(SERVICE_GET_MAP_AREA_EDGE, &NavitMapServer::getMapAreaEdgeCallback, this);

  rasterize_polygon_server_ = nh_.advertiseService("/navit/build_grid_map",
                                                    &NavitMapServer::rasterizePolygonCallback, this);
  switch_map_server_ = nh_.advertiseService("/navit/switch_map", &NavitMapServer::switchMapCallback, this);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

  google::protobuf::util::JsonParseOptions json_parse_option;
  json_parse_option.ignore_unknown_fields = true;
  total_map_info_proto_.Clear();
  auto status =
      google::protobuf::util::JsonStringToMessage(total_map_info_string_, &total_map_info_proto_, json_parse_option);
  map_ok_ = status.ok() && (total_map_info_proto_.map_infos_size() > 0);
  if (!map_ok_)
  {
    printf("map parse failed with file %s.\n", map_file_.c_str());
  }

  switchMap(0);
}

void NavitMapServer::update()
{
  update_publish();
  update_rviz_display();
}
void NavitMapServer::update_publish() const
{
  /// publish(forbidden_line_msg);
  navit_msgs::LineArray forbidden_line_msg;
  for (auto& map_forbidden_line : current_map_info_proto_.map_forbidden_lines())
  {
    auto& line = forbidden_line_msg.lines.emplace_back();
    for (auto& map_point : map_forbidden_line.path())
    {
      line.points.emplace_back(convert(map_point));
    }
  }
  map_forbidden_line_pub_.publish(forbidden_line_msg);

  /// publish(forbidden_areas);
  navit_msgs::PolygonArray forbidden_area_msg;
  for (auto& map_area : current_map_info_proto_.map_areas())
  {
    if (map_area.type() != MapArea::AREA_TYPE_FORBIDDEN_AREA)
    {
      continue;
    }

    auto& forbidden_area = forbidden_area_msg.polygon_array.emplace_back();
    for (auto& map_point : map_area.path())
    {
      forbidden_area.points.emplace_back(convert(map_point));
    }
  }
  map_forbidden_area_pub_.publish(forbidden_area_msg);

  /// map_info_pub_.publish(map_info_msg);
  std_msgs::String map_info_msg;
  map_info_msg.data = current_map_info_string_;
  map_info_pub_.publish(map_info_msg);
}

void NavitMapServer::update_rviz_display()
{
  /// update_rviz_display
  clear_markers();
  visualization_msgs::MarkerArray markerArray;

  int id = 0;
  visualization_msgs::Marker base_marker;
  base_marker.id = ++id;
  base_marker.ns = "none";
  base_marker.header.frame_id = "map";
  base_marker.pose.orientation.w = 1;
  base_marker.scale.x = 0.02;
  base_marker.scale.y = 0.02;
  base_marker.scale.z = 0.0001;
  base_marker.color.a = 1;

  // map_points
  for (auto& station : current_map_info_proto_.map_points())
  {
    base_marker.color.r = 0;
    base_marker.color.g = 0.5;
    base_marker.color.b = 0;
    draw_station(markerArray, station, id, base_marker);
  }

  // map_forbidden_lines
  for (auto& line : current_map_info_proto_.map_forbidden_lines())
  {
    base_marker.color.r = 1;
    base_marker.color.g = 0;
    base_marker.color.b = 0;
    draw_forbidden_line(markerArray, line, id, base_marker);
  }

  // map_lines
  for (auto& curve : current_map_info_proto_.map_lines())
  {
    base_marker.color.r = 1;
    base_marker.color.g = 0.38;
    base_marker.color.b = 0;
    draw_curve(markerArray, curve, id, base_marker);
  }

  // map_areas
  for (auto& area : current_map_info_proto_.map_areas())
  {
    if (area.type() == MapArea::AREA_TYPE_FORBIDDEN_AREA)  // map_forbidden_areas
    {
      base_marker.color.r = 1;
      base_marker.color.g = 0;
      base_marker.color.b = 0;
      draw_polygon(markerArray, area, id, base_marker);
    }
    else
    {
      base_marker.color.r = 1;
      base_marker.color.g = 0.5;
      base_marker.color.b = 0.9;
      draw_polygon(markerArray, area, id, base_marker);
    }
  }

  map_info_display_pub_.publish(markerArray);
}

/***********************************************************************/
/***********************************************************************/
/***********************************************************************/

void NavitMapServer::onSwitchMap(const std_msgs::String::ConstPtr& msg)
{
  for (int i = 0; i < total_map_info_proto_.map_infos_size(); ++i)
  {
    auto& map_info = total_map_info_proto_.map_infos(i);
    if (map_info.map_basic_info().map_name() == msg->data)
    {
      switchMap(i);
      return;
    }
  }
}

void NavitMapServer::switchMap(int map_index)
{
  current_map_info_index_ = map_index;
  current_map_info_string_.clear();
  current_map_info_proto_ = total_map_info_proto_.map_infos(current_map_info_index_);
  google::protobuf::util::MessageToJsonString(current_map_info_proto_, &current_map_info_string_);
  current_map_station_id_name_map_.clear();
  for (auto& station : current_map_info_proto_.map_points())
  {
    current_map_station_id_name_map_[station.id()] = station.name();
  }
  update();
}

void NavitMapServer::onMapInfoUpdate(const std_msgs::String::ConstPtr& msg)
{
  MapInfo temp_current_map_info_proto_{};
  google::protobuf::util::JsonParseOptions json_parse_option;
  json_parse_option.ignore_unknown_fields = true;
  auto status =
      google::protobuf::util::JsonStringToMessage(msg->data, &temp_current_map_info_proto_, json_parse_option);
  if (status.ok())
  {
    // current_map_info
    current_map_info_string_ = msg->data;
    current_map_info_proto_ = temp_current_map_info_proto_;
    current_map_station_id_name_map_.clear();
    for (auto& station : current_map_info_proto_.map_points())
    {
      current_map_station_id_name_map_[station.id()] = station.name();
    }
    // total_map_info
    total_map_info_string_.clear();
    total_map_info_proto_.mutable_map_infos(current_map_info_index_)->CopyFrom(current_map_info_proto_);
    google::protobuf::util::MessageToJsonString(total_map_info_proto_, &total_map_info_string_);
    update();
  }
}

bool NavitMapServer::saveMapInfoCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (!map_ok_)
  {
    return false;
  }
  return save_file(map_file_, total_map_info_string_);
}

bool NavitMapServer::getMapStationPoseCallback(navit_msgs::GetMapStationPose::Request& req,
                                               navit_msgs::GetMapStationPose::Response& res)
{
  if (!map_ok_)
  {
    return false;
  }

  auto* station = get_station(req.station_name);
  if (station)
  {
    res.pose = convertToPoseStamped(station->point()).pose;
    return true;
  }

  return false;
}

bool NavitMapServer::getMapRoutePathCallback(navit_msgs::GetMapRoutePath::Request& req,
                                             navit_msgs::GetMapRoutePath::Response& res)
{
  if (!map_ok_)
  {
    return false;
  }
  return generate_route_path(req.waypoints, req.distance_interval, res.path);
}

bool NavitMapServer::getMapAreaEdgeCallback(navit_msgs::GetMapAreaEdge::Request& req,
                                            navit_msgs::GetMapAreaEdge::Response& res)
{
  if (!map_ok_)
  {
    return false;
  }

  for (auto& area : current_map_info_proto_.map_areas())
  {
    if (area.name() == req.area_name)
    {
      for (auto& area_pose : area.path())
      {
        res.path.poses.push_back(convertToPoseStamped(area_pose));
      }
      return true;
    }
  }
  return false;
}

geometry_msgs::PoseStamped NavitMapServer::convertToPoseStamped(const Point& point)
{
  geometry_msgs::PoseStamped poseStamped;

  poseStamped.pose.position.x = point.x();
  poseStamped.pose.position.y = point.y();
  poseStamped.pose.position.z = point.z();

  tf2::Quaternion quat;
  quat.setRPY(point.rx(), point.ry(), point.rz());

  poseStamped.pose.orientation.x = quat.x();
  poseStamped.pose.orientation.y = quat.y();
  poseStamped.pose.orientation.z = quat.z();
  poseStamped.pose.orientation.w = quat.w();
  return poseStamped;
}

/***********************************************************************/
/***********************************************************************/
/***********************************************************************/

void NavitMapServer::clear_markers()
{
  visualization_msgs::MarkerArray markerArray;
  auto& marker = markerArray.markers.emplace_back();
  marker.action = visualization_msgs::Marker::DELETEALL;
  map_info_display_pub_.publish(markerArray);
}

void NavitMapServer::draw_station(visualization_msgs::MarkerArray& markerArray, const MapPoint& station, int& id,
                                  visualization_msgs::Marker& base_marker)
{
  auto& marker = markerArray.markers.emplace_back(base_marker);
  marker.id = ++id;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.pose.position.x = station.point().x();
  marker.pose.position.y = station.point().y();
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.001;

  /// text_marker
  auto& text_marker = markerArray.markers.emplace_back(base_marker);
  text_marker.id = ++id;
  text_marker.text = station.name();
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.pose = marker.pose;
  text_marker.pose.position.y += 0.1;
  text_marker.scale.x = 0.1;
  text_marker.scale.y = 0.1;
  text_marker.scale.z = 0.2;
  text_marker.color = marker.color;
}

void NavitMapServer::draw_forbidden_line(visualization_msgs::MarkerArray& markerArray, const MapForbiddenLine& line,
                                         int& id, visualization_msgs::Marker& base_marker)
{
  auto& marker = markerArray.markers.emplace_back(base_marker);
  marker.id = ++id;
  marker.type = visualization_msgs::Marker::LINE_LIST;

  for (auto& p : line.path())
  {
    auto& pose = marker.points.emplace_back();
    pose.x = p.x();
    pose.y = p.y();
  }
}

void NavitMapServer::draw_curve(visualization_msgs::MarkerArray& markerArray, const MapCurve& curve, int& id,
                                visualization_msgs::Marker& base_marker)
{
  auto& marker = markerArray.markers.emplace_back(base_marker);
  marker.id = ++id;
  marker.type = visualization_msgs::Marker::POINTS;

  nav_msgs::Path path;
  if (generate_curve_path(&curve, 0.01, &path))
  {
    for (auto& p : path.poses)
    {
      auto& point = marker.points.emplace_back();
      point.x = p.pose.position.x;
      point.y = p.pose.position.y;
    }
  }
}

void NavitMapServer::draw_polygon(visualization_msgs::MarkerArray& markerArray, const MapArea& area, int& id,
                                  visualization_msgs::Marker& base_marker)
{
  auto& marker = markerArray.markers.emplace_back(base_marker);
  marker.id = ++id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  for (auto& area_pose : area.path())
  {
    auto& point = marker.points.emplace_back();
    point.x = area_pose.x();
    point.y = area_pose.y();
  }
  auto first_point = marker.points.front();
  marker.points.emplace_back(first_point);

  /// text_marker
  auto& text_marker = markerArray.markers.emplace_back(base_marker);
  text_marker.id = ++id;
  text_marker.text = area.name();
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.pose.position.x = first_point.x;
  text_marker.pose.position.y = first_point.y + 0.1;
  text_marker.scale.x = 0.1;
  text_marker.scale.y = 0.1;
  text_marker.scale.z = 0.2;
  text_marker.color = marker.color;
}

/***********************************************************************/
/***********************************************************************/
/***********************************************************************/

const NavitMapServer::MapPoint* NavitMapServer::get_station(const std::string& station_name)
{
  for (auto& pos : current_map_info_proto_.map_points())
  {
    if (pos.name() == station_name)
    {
      return &pos;
    }
  }
  return nullptr;
}

const NavitMapServer::MapPoint* NavitMapServer::get_station(const int64_t& station_id)
{
  for (auto& pos : current_map_info_proto_.map_points())
  {
    if (pos.id() == station_id)
    {
      return &pos;
    }
  }
  return nullptr;
}

std::string NavitMapServer::get_station_name(const int64_t& station_id)
{
  if (current_map_station_id_name_map_.count(station_id) == 0)
  {
    return "";
  }

  return current_map_station_id_name_map_[station_id];
}

const NavitMapServer::MapCurve* NavitMapServer::get_curve(const std::string& start, const std::string& end)
{
  for (auto& curve : current_map_info_proto_.map_lines())
  {
    auto start_station_name = get_station_name(curve.start_point_id());
    auto end_station_name = get_station_name(curve.end_point_id());
    if ((start_station_name == start) && (end_station_name == end))
    {
      return &curve;
    }
  }
  return nullptr;
}

bool NavitMapServer::generate_route_path(const std::vector<std::string>& via_point_order, double distance_interval,
                                         nav_msgs::Path& path)
{
  for (int i = 0; i < via_point_order.size() - 1; ++i)
  {
    auto* curve = get_curve(via_point_order[i], via_point_order[i + 1]);
    if (!curve || (generate_curve_path(curve, distance_interval, &path) <= 0))
    {
      return false;
    }
  }
  return true;
}

double NavitMapServer::generate_curve_path(const MapCurve* curve, double distance_interval, nav_msgs::Path* out_path)
{
  // 贝塞尔曲线:BezierPath
  if (curve->type() == MapCurve::LineType::MapLine_LineType_LINE_TYPE_BEZIER)
  {
    return generate_bezier_path(curve, distance_interval, out_path);
  }

  // 圆弧:ArcPath
  if (curve->type() == MapCurve::LineType::MapLine_LineType_LINE_TYPE_ARC)
  {
    return generate_arc_path(curve, distance_interval, out_path);
  }

  // 直线:StraightPath
  if (curve->type() == MapCurve::LineType::MapLine_LineType_LINE_TYPE_DISCRETE)
  {
    return generate_straight_path(curve, distance_interval, out_path);
  }

  return -1;
}

double NavitMapServer::generate_bezier_path(const MapCurve* curve, double distance_interval, nav_msgs::Path* out_path)
{
  auto* start_pose = get_station(curve->start_point_id());
  auto* end_pose = get_station(curve->end_point_id());
  if (!start_pose || !end_pose)
  {
    return -1;
  }

  std::vector<Bezier::Point> bezier_control_points;
  /// 1
  {
    Bezier::Point point;
    point.x = start_pose->point().x();
    point.y = start_pose->point().y();
    bezier_control_points.push_back(point);
  }
  /// 2
  {
    Bezier::Point point;
    point.x = curve->path(0).x();
    point.y = curve->path(0).y();
    bezier_control_points.push_back(point);
  }
  /// 3
  {
    Bezier::Point point;
    point.x = curve->path(1).x();
    point.y = curve->path(1).y();
    bezier_control_points.push_back(point);
  }
  /// 4
  {
    Bezier::Point point;
    point.x = end_pose->point().x();
    point.y = end_pose->point().y();
    bezier_control_points.push_back(point);
  }

  Bezier::Bezier<3> bezier3(bezier_control_points);

  auto distance = bezier3.length();
  int sampling_point_num = std::ceil(distance / distance_interval);
  const float dt = 1.0f / (float)sampling_point_num;
  if (!out_path)
  {
    return distance;
  }

  for (float t = 0; t < 1; t += dt)
  {
    auto point = bezier3.valueAt(t);
    auto& p = out_path->poses.emplace_back();
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = point.x;
    p.pose.position.y = point.y;
    p.pose.orientation.w = 1;
  }

  /// end_pose
  {
    auto& p = out_path->poses.emplace_back();
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = end_pose->point().x();
    p.pose.position.y = end_pose->point().y();

    tf2::Quaternion quat;
    quat.setRPY(0, 0, end_pose->point().rz());

    geometry_msgs::Quaternion quaternion;
    quaternion.x = quat.x();
    quaternion.y = quat.y();
    quaternion.z = quat.z();
    quaternion.w = quat.w();

    p.pose.orientation = quaternion;
  }

  return distance;
}

double NavitMapServer::generate_arc_path(const MapCurve* curve, double distance_interval, nav_msgs::Path* out_path)
{
  double distance = -1;
  if (!out_path)
  {
    return distance;
  }

  return -1;
}

double NavitMapServer::generate_straight_path(const MapCurve* curve, double distance_interval, nav_msgs::Path* out_path)
{
  auto* start_pose = get_station(curve->start_point_id());
  auto* end_pose = get_station(curve->end_point_id());
  if (!start_pose || !end_pose)
  {
    return -1;
  }

  auto& start = start_pose->point();
  auto& end = end_pose->point();
  auto diff_x = end.x() - start.x();
  auto diff_y = end.y() - start.y();
  auto yaw = atan2(diff_y, diff_x);

  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);

  geometry_msgs::Quaternion quaternion;
  quaternion.x = quat.x();
  quaternion.y = quat.y();
  quaternion.z = quat.z();
  quaternion.w = quat.w();

  auto distance = hypot(end.y() - start.y(), end.x() - start.x());
  int sampling_point_num = std::ceil(distance / distance_interval);
  const float dt = 1.0f / (float)sampling_point_num;

  if (!out_path)
  {
    return distance;
  }

  for (float t = 0; t < 1; t += dt)
  {
    auto& p = out_path->poses.emplace_back();
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = start.x() + t * diff_x;
    p.pose.position.y = start.y() + t * diff_y;
    p.pose.orientation = quaternion;
  }

  /// end_pose
  {
    auto& p = out_path->poses.emplace_back();
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = end.x();
    p.pose.position.y = end.y();
    p.pose.orientation = quaternion;
  }

  return distance;
}

bool NavitMapServer::rasterizePolygonCallback(navit_msgs::BuildGridMap::Request &req,
                                              navit_msgs::BuildGridMap::Response &res) {
  if (req.polygons.empty()) {
    res.success = false;
    return true;
  }
  // print polygon points
  std::vector<std::vector<cv::Point2f>> drivable_polygons;
  std::vector<std::vector<cv::Point2f>> non_drivable_polygons;

  for (int i = 0; i < req.polygons.size(); ++i) {
    std::vector<cv::Point2f> points;
    for (const auto& point : req.polygons[i].points) {
      points.emplace_back((point.x), (point.y));
    }
    if (i == 0) {
      drivable_polygons.push_back(points);
      if (drivable_polygons.back().back() != drivable_polygons.back().front()) {
        ROS_INFO("Add polygon end point to polygon.");
        drivable_polygons.back().push_back(drivable_polygons.back().front());
      }

    } else {
      non_drivable_polygons.push_back(points);
      if (non_drivable_polygons.back().back() != non_drivable_polygons.back().front()) {
        non_drivable_polygons.back().push_back(non_drivable_polygons.back().front());
      }
    }
  }

  // holes can be outside of the drivable area, so we need to merge them
  std::vector<cv::Point2f> all_points;
  for (const auto& poly : drivable_polygons) {
    all_points.insert(all_points.end(), poly.begin(), poly.end());
  }
  for (const auto& poly : non_drivable_polygons) {
    all_points.insert(all_points.end(), poly.begin(), poly.end());
  }

  cv::Rect2f bounding_box = cv::boundingRect(all_points);
  //TODO(czk): add param
  float resolution = 0.05;
  cv::Size image_size(std::ceil(bounding_box.width / resolution), std::ceil(bounding_box.height / resolution));
  cv::Mat image = cv::Mat::ones(image_size, CV_8UC1) * 0;

  //print drivable_polygons
  for (int i = 0; i < non_drivable_polygons.size(); ++i) {
    ROS_INFO("non  %d:", i);
    for (const auto& point : non_drivable_polygons[i]) {
      ROS_INFO("x: %f, y: %f", point.x, point.y);
    }
  }
  //print non_drivable_polygons

  std::vector<std::vector<cv::Point>> boundary_pixels, forbidden_pixels, lines_pixels;
  for (const auto& polygons : {std::make_pair(drivable_polygons, &boundary_pixels),
                                std::make_pair(non_drivable_polygons, &forbidden_pixels)}) {
        if (polygons.first.empty()) {
            continue;
        }
        for (const auto& polygon : polygons.first) {
            std::vector<cv::Point> pixel_points;
            for (const auto& point : polygon) {
                pixel_points.push_back(cv::Point(std::round((point.x - bounding_box.x) / resolution),
                                                std::round((point.y - bounding_box.y) / resolution)));
            }
            polygons.second->push_back(pixel_points);
        }
  }

  cv::fillPoly(image, boundary_pixels, cv::Scalar(100));
  cv::fillPoly(image, forbidden_pixels, cv::Scalar(0));

  // 保存图像和 YAML 文件
  std::string image_name = req.label + ".png";
  std::string yaml_name = req.label + ".yaml";

  cv::Point origin(bounding_box.x/resolution, bounding_box.y/resolution);

  std::string full_path = map_dir_ + "/" + image_name;
  cv::imwrite(full_path, image);

  bool result = saveMapMetadata(map_dir_, yaml_name, image_name, resolution, origin, image_size);
  if (result) {
    res.success = true;
    ROS_INFO("Save map success.");
  } else {
    res.success = false;
    ROS_INFO("Save map failed.");
  }
  //转换为grid map 并 发布
  nav_msgs::OccupancyGrid grid_msg;
  if (loadMapMetadata(map_dir_ + "/" + yaml_name, grid_msg)) {
    ROS_INFO("Load map success.");
    grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);
    for (int i = 0; i < grid_msg.info.height; i++) {
      for (int j = 0; j < grid_msg.info.width; j++) {
        grid_msg.data[i * grid_msg.info.width + j] = image.at<uint8_t>(i, j) > 0 ? 0 : 100;
      }
    }
    ROS_INFO("Load map success.");
    map_pub_.publish(grid_msg);
  }
  return true;
}

bool NavitMapServer::saveMapMetadata(const std::string& mapDirectory, const std::string& yamlFilename,
                                     const std::string& imageFilename, float resolution,
                                     const cv::Point& origin, const cv::Size& imageSize)
{
  std::string yamlPath = mapDirectory + "/" + yamlFilename;

  std::ofstream yamlFile(yamlPath);
  if (!yamlFile)
  {
    std::cerr << "Unable to open file for writing: " << yamlPath << std::endl;
    return false;
  }

  // 使用相对路径或仅文件名来引用图像文件
  yamlFile << "image: " << imageFilename << std::endl;
  yamlFile << "resolution: " << resolution << std::endl;
  yamlFile << "origin: [" << origin.x * resolution << ", " << origin.y * resolution << ", 0.0]" << std::endl;
  yamlFile << "negate: 0" << std::endl;
  yamlFile << "occupied_thresh: 0.65" << std::endl;
  yamlFile << "free_thresh: 0.196" << std::endl;
  yamlFile << "width: " << imageSize.width << std::endl;
  yamlFile << "height: " << imageSize.height << std::endl;

  yamlFile.close();
  return true;
}

bool NavitMapServer::switchMapFile(const std::string& map_dir, const std::string& map_name) {
  std::string image_file_path = map_dir + "/" + map_name + ".png";
  std::string yaml_file_path = map_dir + "/" + map_name + ".yaml";

  if (!boost::filesystem::exists(image_file_path) || !boost::filesystem::exists(yaml_file_path)) {
    ROS_ERROR("Failed to find map file: %s", image_file_path.c_str());
    return false;
  }

  cv::Mat image = cv::imread(image_file_path, cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    ROS_ERROR("Failed to load map image file: %s", image_file_path.c_str());
    return false;
  }

  nav_msgs::OccupancyGrid grid_msg;
  // 读取yaml 文件，填充 gridMsg

  if (!loadMapMetadata(yaml_file_path, grid_msg)) {
    ROS_ERROR("Failed to load map yaml file: %s", yaml_file_path.c_str());
    return false;
  }

  grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);
  for (int i = 0; i < grid_msg.info.height; i++) {
    for (int j = 0; j < grid_msg.info.width; j++) {
      grid_msg.data[i * grid_msg.info.width + j] = image.at<uint8_t>(i, j) > 0 ? 0 : 100;
    }
  }

  map_pub_.publish(grid_msg);
  return true;
}

bool NavitMapServer::loadMapMetadata(const std::string& yaml_file_path, nav_msgs::OccupancyGrid& grid_msg) {
    std::ifstream yamlFile(yaml_file_path);
    if (!yamlFile) {
        std::cerr << "Unable to open file for reading: " << yaml_file_path << std::endl;
        return false;
    }

    // Load the YAML file
    YAML::Node config = YAML::Load(yamlFile);

    grid_msg.header.frame_id = "map";
    grid_msg.header.stamp = ros::Time::now();

    try {
        grid_msg.info.width = config["width"].as<unsigned int>();
        grid_msg.info.height = config["height"].as<unsigned int>();
        grid_msg.info.resolution = config["resolution"].as<float>();

        if (config["origin"]) {
            grid_msg.info.origin.position.x = config["origin"][0].as<double>();
            grid_msg.info.origin.position.y = config["origin"][1].as<double>();
            grid_msg.info.origin.position.z = config["origin"][2].as<double>();
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parse error: " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool NavitMapServer::switchMapCallback(navit_msgs::SwitchTaskMap::Request &req,
                                       navit_msgs::SwitchTaskMap::Response &res) {
  std::string new_map_name = req.map_name;

  if (!switchMapFile(map_dir_, req.map_name)) {
    res.success = false;
    return false;
  }
  res.success = true;
  return true;
}

}  // namespace navit_map_server
