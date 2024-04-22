#pragma once
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMaterial.h>
#include <OgreManualObject.h>
#include <OgreVector3.h>
#include <OgreOverlayManager.h>

#include <Ogre.h>
#include <OgreOverlay.h>
#include <OgreOverlayContainer.h>
#include <OgreOverlayElement.h>
#include <OgreOverlayManager.h>
#include <OgreTextAreaOverlayElement.h>

#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/properties/float_property.h>
#include <rviz/tool.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>

#include <chrono>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "message_navit_map.pb.h"

namespace rviz_combined_tool
{
struct TypedPolygon
{
  navit::protocol::map_info::MapArea propertyType;
  std::vector<Ogre::Vector3> vertices;
};

struct TypedLine
{
  navit::protocol::map_info::MapLine propertyType;
  std::vector<Ogre::Vector3> vertices;
};

struct TypedPoint
{
  navit::protocol::map_info::MapPoint propertyType;
  Ogre::Vector3 vertex;
  Ogre::Vector3 euler;
};

enum class DrawType
{
  NONE,
  DRAW,
  SELECT_AREA,
  SELECT_LINE,
  SELECT_POINT
};
enum LineType {
  LINE_TYPE_NORMAL,
  LINE_TYPE_TEACHING,
};
class CombinedSelectionTool : public rviz::Tool
{
  Q_OBJECT

public:
  CombinedSelectionTool();

  void onInitialize() override;
  void activate() override {};
  void deactivate() override {};
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

  bool setItemProperty(const navit::protocol::map_info::ItemProperty item_property) {
    current_property_ = item_property;
    return true;
  }
  // 
  bool savePolygon();
  bool saveLine();
  bool savePoint();
  //
  bool getMap();
  bool setMap();
  
  bool getMap(navit::protocol::map_info::MapInfo& map_info);
  bool setMap(const navit::protocol::map_info::MapInfo map_info);
  
  bool expandMapPolygon(const navit::protocol::map_info::MapInfo map_info);
  bool expandMapLine(const navit::protocol::map_info::MapInfo map_info);

  bool clearMap();

  bool selectArea() { draw_type_ = DrawType::SELECT_AREA; return true;}

  bool selectLine() { draw_type_ = DrawType::SELECT_LINE; return true; }

  bool selectPoint() { draw_type_ = DrawType::SELECT_POINT; return true; }

  bool getSelectArea(std::vector<std::string>& polygons_id);

  bool getSelectLine(std::vector<std::string>& lines);

  bool getSelectPoint(std::vector<navit::protocol::map_info::Point>& points);
  
public Q_SLOTS:
  void updatePtsSize();
  void updateVisual();

private:
  const float SELECTION_THRESHOLD = 0.3f;

  Ogre::Vector3 findNearbyPoint(const Ogre::Vector3& pos, float threshold);
 
 
  void clearHighlights();
  void clearPointHighlights();
  void clearLineHighlights();
  void unhighlightAll();
  
  void unhighlightPolygon(int index);
  void toggleHighlightPolygon(const TypedPolygon& polygon, int index);
  void toggleHighlightLine(const Ogre::Vector3& start, const Ogre::Vector3& end, int index);
  std::vector<Ogre::Vector3> triangulatePolygon(const TypedPolygon& polygon);

  bool isPolygonSelected(const TypedPolygon& polygon, const rviz::ViewportMouseEvent& event);
  bool isLineSegmentSelected(const Ogre::Vector3& start, const Ogre::Vector3& end, const Ogre::Vector3& even);
  bool isPointSelected(const Ogre::Vector3& point, const Ogre::Vector3& event);

  void toggleHighlightPoint(const Ogre::Vector3& point, int index);
  void unhighlightLineSegment(int index);
  void highlightPolygon(const TypedPolygon& polygon, int index);
  void highlightLineSegment(const Ogre::Vector3& start, const Ogre::Vector3& end, int index);
  void highlightPoint(const Ogre::Vector3& point, int index);
  void unhighlightPoint(int index);
  float distanceToLineSegment(const Ogre::Vector2& point, const Ogre::Vector2& lineStart, const Ogre::Vector2& lineEnd);
  Ogre::ManualObject* createLineSegment(const Ogre::Vector3& start, const Ogre::Vector3& end, float thickness, const std::string& materialName);
  Ogre::ManualObject* createGeometry(const std::vector<std::pair<Ogre::Vector3, Ogre::Vector3>>& segments, float thickness, const std::string& materialName);
  Ogre::ManualObject* createHighlightPoint(const Ogre::Vector3& point, float size, const std::string& materialName);
  std::vector<Ogre::Vector3> calculateStarVertices(const Ogre::Vector3& center, float size);
  void createText(const std::string& text, const Ogre::Vector3& position, const std::string& id);
  void addLineSegment(const Ogre::Vector3& start, const Ogre::Vector3& end, LineType type);
  Ogre::Vector2 convertWorldToScreenPosition(const Ogre::Vector3& worldPos, Ogre::Camera* camera);
  void publishShapes(const std::vector<TypedPoint>& typed_points,
                    const std::vector<TypedLine>& typed_lines,
                    const std::vector<TypedPolygon>& typed_polygons,
                    const ros::Publisher& points_marker_pub,
                    const ros::Publisher& lines_marker_pub,
                    const ros::Publisher& polygons_marker_pub,
                    const std::string& frame_id);

  geometry_msgs::Point convertToPoint(const navit::protocol::map_info::Point& proto_point) {
    geometry_msgs::Point point;
    point.x = proto_point.x();
    point.y = proto_point.y();
    point.z = proto_point.z();
    return point;
  }
  std::vector<geometry_msgs::Point> convertLineToPoints(const TypedLine& typed_line) {
    std::vector<geometry_msgs::Point> points;
    for (const auto& vertex : typed_line.vertices) {
        geometry_msgs::Point point;
        point.x = vertex.x;
        point.y = vertex.y;
        point.z = vertex.z;
        points.push_back(point);
    }
    return points;
  }
  std::vector<geometry_msgs::Point> convertPolygonToPoints(const TypedPolygon& typed_polygon) {
      std::vector<geometry_msgs::Point> points;
      for (const auto& vertex : typed_polygon.vertices) {
          geometry_msgs::Point point;
          point.x = vertex.x;
          point.y = vertex.y;
          point.z = vertex.z;
          points.push_back(point);
      }
      // Ensure the polygon is closed by repeating the first vertex at the end
      if (!points.empty()) {
          points.push_back(points.front());
      }
      return points;
  }
  geometry_msgs::Point convertPointToGeomPoint(const TypedPoint& typed_point) {
    geometry_msgs::Point point;
    point.x = typed_point.vertex.x;
    point.y = typed_point.vertex.y;
    point.z = typed_point.vertex.z;
    return point;
  }

  void clearAllMarkers(const ros::Publisher& marker_pub, const std::string& ns) {
      visualization_msgs::MarkerArray clear_array;
      visualization_msgs::Marker clear_marker;
      clear_marker.header.frame_id = "frame_id"; // Use your actual frame_id
      clear_marker.header.stamp = ros::Time::now();
      clear_marker.ns = ns;
      clear_marker.action = visualization_msgs::Marker::DELETEALL;
      clear_array.markers.push_back(clear_marker);
      marker_pub.publish(clear_array);
  }

  rviz::BoolProperty* lasso_mode_property_;
  rviz::BoolProperty* close_loop_property_;
  rviz::FloatProperty* pt_size_property_;
  rviz::FloatProperty* line_size_property_;

  Ogre::ManualObject* pts_vis_;
  Ogre::ManualObject* line_vis_;

  Ogre::ManualObject* polygon_vis_;

  Ogre::MaterialPtr pts_material_;
  Ogre::MaterialPtr line_material_;
  Ogre::MaterialPtr polygon_material_;
  Ogre::MaterialPtr hightlight_material_;


  Ogre::MaterialPtr red_material_;
  Ogre::MaterialPtr blue_material_;
  Ogre::MaterialPtr yellow_material_;

  Ogre::Vector3 start_point_;
  bool is_start_point_selected_ = false;
  
  // for points  
  std::vector<Ogre::Vector3> points_;
  std::vector<TypedPoint> typed_points_;
  // for lines
  std::vector<Ogre::Vector3> lines_, teaching_points_;
  std::vector<TypedLine> typed_lines_;
  // for polygons
  std::vector<Ogre::Vector3> vertices_;
  std::vector<TypedPolygon> polygons_;

  navit::protocol::map_info::ItemProperty current_property_;
  std::chrono::steady_clock::time_point last_click_time_;

  // std::vector<TypedPolygon> select_polygons_;
  std::unordered_map<size_t, TypedPolygon> select_polygons_;
  std::unordered_map<int, Ogre::ManualObject*> highlightedPolygonObjects_, highlightedLineObjects_, highlightedPointObjects_;
  std::unordered_map<size_t, TypedLine> select_lines_;
  std::unordered_map<size_t, TypedPoint> select_points_;

  std::unordered_set<int> highlightedPolygonIndices_;
  std::unordered_set<int> highlightedLineIndices_;
  DrawType draw_type_ = DrawType::NONE;

  Ogre::Vector3 point_direction_;
  ros::Publisher polygons_marker_pub_, lines_marker_pub_, points_marker_pub_, names_marker_pub_;
};
}  // namespace rviz_combined_tool

