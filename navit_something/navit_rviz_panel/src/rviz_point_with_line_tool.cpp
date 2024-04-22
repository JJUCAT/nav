#include <navit_rviz_panel/rviz_point_with_line_tool.h>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
namespace rviz_combined_tool
{

CombinedSelectionTool::CombinedSelectionTool()
{
  points_.clear();
  lines_.clear();
  shortcut_key_ = 'c';
}

void CombinedSelectionTool::onInitialize()
{
  ros::NodeHandle nh;
  points_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/navit_rviz_panel/points_marker", 10);
  lines_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/navit_rviz_panel/lines_marker", 10);
  polygons_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/navit_rviz_panel/polygons_marker", 10);
  names_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/navit_rviz_panel/names_marker", 10);


  polygon_vis_ = scene_manager_->createManualObject("lines");
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(polygon_vis_);
  polygon_material_ = Ogre::MaterialManager::getSingleton().create("lines_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  pts_vis_ = scene_manager_->createManualObject("work_points");
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(pts_vis_);

  if (pts_material_.isNull()) {
    pts_material_ = Ogre::MaterialManager::getSingleton().create("points_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    pts_material_->setDiffuse(Ogre::ColourValue(1, 0, 0, 1));
    pts_material_->setAmbient(Ogre::ColourValue(1, 0, 0, 1));
    pts_material_->setLightingEnabled(false);
  }

  if (line_material_.isNull()) {
    line_material_ = Ogre::MaterialManager::getSingleton().create("lines_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    line_material_->setDiffuse(Ogre::ColourValue(1, 0, 0, 1));
    line_material_->setAmbient(Ogre::ColourValue(1, 0, 0, 1));
    line_material_->setLightingEnabled(false);
  }

  if (hightlight_material_.isNull()) {
    hightlight_material_ = Ogre::MaterialManager::getSingleton().create("hightlight_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    hightlight_material_->setDiffuse(Ogre::ColourValue(1, 0, 0, 1));
    hightlight_material_->setAmbient(Ogre::ColourValue(1, 0, 0, 1));
    hightlight_material_->setLightingEnabled(false);
  }

  line_vis_ = scene_manager_->createManualObject("connection_line");
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(line_vis_);
  pt_size_property_ = new rviz::FloatProperty("Point Size", 15.0, "Size of clicked points", getPropertyContainer(), SLOT(updatePtsSize()), this);
  line_size_property_ = new rviz::FloatProperty("Line Size", 5.0, "Size of clicked lines", getPropertyContainer(), SLOT(updatePtsSize()), this);


  close_loop_property_ = new rviz::BoolProperty("Close loop", true, "Close the polygon with a line between the last and " "first points", getPropertyContainer(), SLOT(updateVisual()), this);
  lasso_mode_property_ = new rviz::BoolProperty("Lasso mode", true, "Toggle between lasso and discrete click mode", getPropertyContainer());

  red_material_ = Ogre::MaterialManager::getSingleton().create("red_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  red_material_->getTechnique(0)->getPass(0)->setDiffuse(1, 0, 0, 0);
  red_material_->getTechnique(0)->getPass(0)->setAmbient(1, 0, 0);
  red_material_->getTechnique(0)->getPass(0)->setSelfIllumination(1, 0, 0);

  blue_material_ = Ogre::MaterialManager::getSingleton().create("blue_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  blue_material_->getTechnique(0)->getPass(0)->setDiffuse(0, 0, 1, 1); // 蓝色
  blue_material_->getTechnique(0)->getPass(0)->setAmbient(0, 0, 1);
  blue_material_->getTechnique(0)->getPass(0)->setSelfIllumination(0, 0, 1);

  updatePtsSize();
}

int CombinedSelectionTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  auto debounce_time = std::chrono::milliseconds(50);
  auto now = std::chrono::steady_clock::now();

  Ogre::Camera* camera = context_->getViewManager()->getCurrent()->getCamera();
  Ogre::Ray mouseRay = camera->getCameraToViewportRay(static_cast<float>(event.x) / event.viewport->getActualWidth(),
                                                      static_cast<float>(event.y) / event.viewport->getActualHeight());
  Ogre::Plane groundPlane(Ogre::Vector3::UNIT_Z, 0.0);

  std::pair<bool, Ogre::Real> intersection = mouseRay.intersects(groundPlane);

  if (event.leftUp() && (draw_type_ == DrawType::SELECT_LINE || draw_type_ == DrawType::SELECT_AREA || draw_type_ == DrawType::SELECT_POINT)) {
    if (intersection.first) // if the ray intersects the plane
    {
      Ogre::Vector3 position = mouseRay.getPoint(intersection.second);
      if (draw_type_ == DrawType::SELECT_LINE) {

        for (size_t i = 0; i < typed_lines_.size(); i++) {
          if (isLineSegmentSelected(typed_lines_[i].vertices[0], typed_lines_[i].vertices[1], position)) {
            select_lines_[i] = typed_lines_[i];
            toggleHighlightLine(typed_lines_[i].vertices[0], typed_lines_[i].vertices[1], i);
          }
        }
        return Render;
      } if (draw_type_ == DrawType::SELECT_AREA) {
        for (size_t i = 0; i < polygons_.size(); i++) {
          if (isPolygonSelected(polygons_[i], event)) {
            select_polygons_[i] = polygons_[i];
            toggleHighlightPolygon(polygons_[i], i);
          }
        }
        return Render;
    } else if (draw_type_ == DrawType::SELECT_POINT) {
        for (size_t i = 0; i < typed_points_.size(); i++) {
          if (isPointSelected(typed_points_[i].vertex, position)) {
            select_points_[i] = typed_points_[i];
            toggleHighlightPoint(typed_points_[i].vertex, i);
          }
        }

        return Render;
      }
    }
  } else if (event.leftUp() || (event.left() && lasso_mode_property_->getBool()) && (draw_type_ == DrawType::NONE))
  {
    if (intersection.first) // if the ray intersects the plane
    {
      Ogre::Vector3 position = mouseRay.getPoint(intersection.second); // get the intersection point

      if(current_property_.item_property() == navit::protocol::map_info::ItemProperty::LINE_TOOL)
      {

        if (current_property_.mutable_map_line()->type() == navit::protocol::map_info::MapLine::LINE_TYPE_TEACHING) {
          if (event.leftUp() && intersection.first) {
            Ogre::Vector3 current_point = mouseRay.getPoint(intersection.second);

            if (teaching_points_.empty() || (teaching_points_.back() - current_point).length() > 0.1) {
              teaching_points_.push_back(current_point);

              if (teaching_points_.size() > 1) {
                addLineSegment(teaching_points_[teaching_points_.size() - 2], current_point, LINE_TYPE_NORMAL);
              }
              updateVisual();
            }
          }
        } else if (current_property_.mutable_map_line()->type() == navit::protocol::map_info::MapLine::LINE_TYPE_DISCRETE) {
          Ogre::Vector3 nearby_point = findNearbyPoint(position, 0.5); // around point 0.5
          if(nearby_point != Ogre::Vector3::ZERO)
          {
            if(!is_start_point_selected_)
            {
              start_point_ = nearby_point;
              is_start_point_selected_ = true;
            }

            if (lines_.empty() && is_start_point_selected_) {
              lines_.push_back(start_point_);
              ROS_INFO("Draw Lines...");
            }

            if (std::find(lines_.begin(), lines_.end(), nearby_point) == lines_.end()) {
              lines_.push_back(nearby_point);
              is_start_point_selected_ = false;
              ROS_INFO("Draw Lines...");
            }
          }
        } else {

        }
      } else if(current_property_.item_property() == navit::protocol::map_info::ItemProperty::POINT_TOOL) {
         if (event.leftUp()) {
            Ogre::Vector3 current_point = mouseRay.getPoint(intersection.second);
            if (points_.empty()) {
                points_.push_back(current_point);
                ROS_INFO("First point set.");
            } else if (points_.size() == 1){
                Ogre::Vector3 direction = (current_point - points_[0]).normalisedCopy();
                float yaw = atan2(direction.y, direction.x);
                float yawDegrees = Ogre::Radian(yaw).valueDegrees();
                point_direction_.x = 0.0f;
                point_direction_.y = 0.0f;
                point_direction_.z = yaw;
                points_.push_back(current_point);
                ROS_INFO("Direction saved.");
            }
          }
        ROS_INFO("Draw points...");
      } else if (current_property_.item_property() == navit::protocol::map_info::ItemProperty::POLYGON_TOOL) {
        if (intersection.first)
        {
          if (vertices_.empty()) {
            vertices_.push_back(position);
            ROS_INFO("Draw Area...");
          }

          if (std::find(vertices_.begin(), vertices_.end(), position) == vertices_.end()) {
            vertices_.push_back(position);
            ROS_INFO("Draw Area...");
          }
        }
      } else {
         ROS_ERROR("Unknown item property.");
      }
      updateVisual();
    }
  }
  else if (event.right())
  {
    if (current_property_.item_property() == navit::protocol::map_info::ItemProperty::POLYGON_TOOL) {
      if (!polygons_.empty()) {
        polygons_.pop_back();
        updateVisual();
        ROS_INFO("Delete Area...");
      }

    } else if (current_property_.item_property() == navit::protocol::map_info::ItemProperty::LINE_TOOL) {
      if (!lines_.empty()) {
        lines_.pop_back();
        lines_.pop_back();
        ROS_INFO("Delete Lines...");
      }
    } else if (current_property_.item_property() == navit::protocol::map_info::ItemProperty::POINT_TOOL) {
      if (!points_.empty()) {
        points_.pop_back();
        ROS_INFO("Delete Points...");
      }
    }
    updateVisual();
  }

  return Render;
}

void CombinedSelectionTool::updatePtsSize()
{
  pts_material_->setPointSize(10);
  line_material_->setPointSize(10);
  hightlight_material_->setPointSize(10);
}

void CombinedSelectionTool::updateVisual()
{

  pts_vis_->clear();
  line_vis_->clear();

  pts_vis_->begin("points_material", Ogre::RenderOperation::OT_POINT_LIST);

  for (int i = 0; i < points_.size(); i++) {
    if (i == 0) {
      pts_vis_->position(points_[i]);
      pts_vis_->colour(Ogre::ColourValue(1, 0, 0, 1));
    } else if (i == 1) {
      pts_vis_->position(points_[i]);
      pts_vis_->colour(Ogre::ColourValue(1, 1, 0, 1));
    }
  }

  for (const auto&type : typed_points_) {
    pts_vis_->position(type.vertex);
  }

  pts_vis_->end();


  line_vis_->begin("lines_material", Ogre::RenderOperation::OT_LINE_LIST);
  for (size_t i = 0; i + 1 < lines_.size(); i += 2) {
    line_vis_->position(lines_[i]);
    line_vis_->position(lines_[i + 1]);
  }

  for (const auto& type : typed_lines_) {
    for (size_t i = 0; i + 1 < type.vertices.size(); i++) {
      line_vis_->position(type.vertices[i]);
      line_vis_->position(type.vertices[i + 1]);
    }
    Ogre::Vector3 label_position = type.vertices[0];
    //createText(type.propertyType.name(), label_position, type.propertyType.name()+ "Label");
  }
  if(current_property_.mutable_map_line()->type() == navit::protocol::map_info::MapLine::LINE_TYPE_TEACHING && !teaching_points_.empty()) {
    for (size_t i = 0; i < teaching_points_.size(); i++) {
      line_vis_->position(teaching_points_[i]);
      if(i + 1 < teaching_points_.size()) {
        line_vis_->position(teaching_points_[i + 1]);
      }
    }
  }
  line_vis_->end();

  polygon_vis_->clear();

  Ogre::MaterialPtr material = red_material_;
  Ogre::MaterialPtr vertex_material = blue_material_;
  if (material.isNull()) {
    ROS_ERROR("Red material is null.");
    return;
  }

  for (const TypedPolygon& polygon : polygons_) {
    if (polygon.vertices.size() > 1) {

      Ogre::Vector3 textPosition = polygon.vertices[0];
      // createText(polygon.propertyType.name(), textPosition, polygon.propertyType.name());

      polygon_vis_->begin(material->getName(), Ogre::RenderOperation::OT_LINE_STRIP);
      for (std::size_t i = 0; i < polygon.vertices.size() - 1; ++i) {
        polygon_vis_->position(polygon.vertices.at(i));
        polygon_vis_->position(polygon.vertices.at(i + 1));
      }

      if (polygon.vertices.size() > 2 && close_loop_property_->getBool()) {
        polygon_vis_->position(polygon.vertices.back());
        polygon_vis_->position(polygon.vertices.front());
      }

      polygon_vis_->end();
    }
  }

  if (vertices_.size() > 1) {
    polygon_vis_->begin(vertex_material->getName(), Ogre::RenderOperation::OT_LINE_STRIP);
    for (std::size_t i = 0; i < vertices_.size() - 1; ++i) {
      polygon_vis_->position(vertices_.at(i));
      polygon_vis_->position(vertices_.at(i + 1));
    }

    if (vertices_.size() > 2 && close_loop_property_->getBool()) {
      polygon_vis_->position(vertices_.back());
      polygon_vis_->position(vertices_.front());
    }

    polygon_vis_->end();
  }
}
Ogre::Vector3 CombinedSelectionTool::findNearbyPoint(const Ogre::Vector3& pos, float threshold)
{
  float min_distance = threshold;
  Ogre::Vector3 nearby_point;
  bool found = false;

  for(const TypedPoint& point : typed_points_)
  {
    float distance = (point.vertex - pos).length();
    if(distance < min_distance)
    {
      min_distance = distance;
      nearby_point = point.vertex;
      found = true;
    }
  }

  if(found) {
    return nearby_point;
  } else {
    return Ogre::Vector3::ZERO;
  }
}

bool CombinedSelectionTool::savePolygon() {
  if (vertices_.size() <= 3) {
    return false;
  }
  polygons_.push_back(TypedPolygon{current_property_.map_polygon(), vertices_});
  vertices_.clear();
  updateVisual();
  publishShapes(typed_points_, typed_lines_, polygons_, points_marker_pub_, lines_marker_pub_, polygons_marker_pub_, "map");
  return true;
}

bool CombinedSelectionTool::saveLine() {
  if (lines_.size() >= 2) {
    is_start_point_selected_ = false;
    typed_lines_.push_back(TypedLine{current_property_.map_line(), lines_});
    lines_.clear();
    updateVisual();
  }
  if (teaching_points_.size() >= 2) {
    is_start_point_selected_ = false;
    typed_lines_.push_back(TypedLine{current_property_.map_line(), teaching_points_});
    teaching_points_.clear();
    updateVisual();
  }
  publishShapes(typed_points_, typed_lines_, polygons_, points_marker_pub_, lines_marker_pub_, polygons_marker_pub_, "map");
  return true;
}

bool CombinedSelectionTool::savePoint() {
  if (points_.size() == 0) {
    return false;
  }
  typed_points_.push_back(TypedPoint{current_property_.map_point(), points_[0], point_direction_});
  points_.clear();
  publishShapes(typed_points_, typed_lines_, polygons_, points_marker_pub_, lines_marker_pub_, polygons_marker_pub_, "map");
  return true;
}

bool CombinedSelectionTool::setMap(const navit::protocol::map_info::MapInfo map_info) {
    clearMap();
    if (map_info.map_areas_size() == 0 && map_info.map_lines_size() == 0 && map_info.map_points_size() == 0) {
      return false;
    }
    for (const navit::protocol::map_info::MapArea &area : map_info.map_areas()) {
      TypedPolygon typedPolygon;
      typedPolygon.propertyType = area;
      for (const navit::protocol::map_info::Point &point : area.path()) {
        Ogre::Vector3 vertex;
        vertex.x = point.x();
        vertex.y = point.y();
        vertex.z = point.z();
        auto it = std::find_if(typedPolygon.vertices.begin(), typedPolygon.vertices.end(), [&vertex](const Ogre::Vector3 &p) {
            return p.x == vertex.x && p.y == vertex.y;
        });

        if (it == typedPolygon.vertices.end()) {
          typedPolygon.vertices.push_back(vertex);
        }
      }
      if (typedPolygon.vertices.size() == 0) {
        continue;
      }
      polygons_.push_back(typedPolygon);
    }

    for (const navit::protocol::map_info::MapLine &line : map_info.map_lines()) {
      TypedLine typedLine;
      typedLine.propertyType = line;
      for (const navit::protocol::map_info::Point &point : line.path()) {
        Ogre::Vector3 vertex;
        vertex.x = point.x();
        vertex.y = point.y();
        vertex.z = point.z();

        auto it = std::find_if(typedLine.vertices.begin(), typedLine.vertices.end(), [&vertex](const Ogre::Vector3 &p) {
            return p.x == vertex.x && p.y == vertex.y;
        });

        if (it == typedLine.vertices.end()) {
          typedLine.vertices.push_back(vertex);
        }
      }
      if (typedLine.vertices.size() == 0) {
        continue;
      }
      typed_lines_.push_back(typedLine);
    }

    for (const navit::protocol::map_info::MapPoint &point : map_info.map_points()) {
      TypedPoint typedPoint;
      typedPoint.propertyType = point;
      Ogre::Vector3 vertex, euler;
      vertex.x = point.point().x();
      vertex.y = point.point().y();
      vertex.z = point.point().z();

      euler.x = point.point().rx();
      euler.y = point.point().ry();
      euler.z = point.point().rz();

      typedPoint.vertex = vertex;
      typedPoint.euler = euler;
      typed_points_.push_back(typedPoint);
    }
    updateVisual();
    return true;
  }

  bool CombinedSelectionTool::expandMapPolygon(const navit::protocol::map_info::MapInfo map_info) {
    //printf polygons_

    for (const navit::protocol::map_info::MapArea &area : map_info.map_areas()) {
      TypedPolygon typedPolygon;
      typedPolygon.propertyType.set_name(area.name());
      typedPolygon.propertyType.set_type(area.type());

      for (const navit::protocol::map_info::Point &point : area.path()) {
        Ogre::Vector3 vertex;
        vertex.x = point.x();
        vertex.y = point.y();
        vertex.z = point.z();
        typedPolygon.vertices.push_back(vertex);

      }
      polygons_.push_back(typedPolygon);
    }
    updateVisual();
    return true;
  }
  bool CombinedSelectionTool::expandMapLine(const navit::protocol::map_info::MapInfo map_info) {

    for (const navit::protocol::map_info::MapLine &line : map_info.map_lines()) {
      TypedLine typedLine;
      typedLine.propertyType.set_name(line.name());
      typedLine.propertyType.set_type(line.type());
      for (const navit::protocol::map_info::Point &point : line.path()) {
        Ogre::Vector3 vertex;
        vertex.x = point.x();
        vertex.y = point.y();
        vertex.z = point.z();
        typedLine.vertices.push_back(vertex);
      }
      typed_lines_.push_back(typedLine);
    }
    updateVisual();
    return true;
  }
  bool CombinedSelectionTool::getMap(navit::protocol::map_info::MapInfo& map_info)
  {
    map_info.clear_map_areas();
    map_info.clear_map_lines();
    map_info.clear_map_points();
    for (const TypedPolygon &typedPolygon : polygons_) {
      navit::protocol::map_info::MapArea area;
      area = typedPolygon.propertyType;
      for (const Ogre::Vector3 &vertex : typedPolygon.vertices) {
        navit::protocol::map_info::Point point;
        point.set_x(vertex.x);
        point.set_y(vertex.y);
        point.set_z(vertex.z);

        auto it = std::find_if(area.path().begin(), area.path().end(), [&point](const navit::protocol::map_info::Point &p) {
          return ((fabs(p.x() - point.x()) < 0.001) && (fabs(p.y() - point.y()) < 0.001));
        });

        if (it == area.path().end()) {
          area.add_path()->CopyFrom(point);
        }
      }
      map_info.add_map_areas()->CopyFrom(area);
    }

    for (const TypedLine &typedLine : typed_lines_) {
      navit::protocol::map_info::MapLine line;
      line = typedLine.propertyType;

      for (const Ogre::Vector3 &vertex : typedLine.vertices) {
        navit::protocol::map_info::Point point;
        point.set_x(vertex.x);
        point.set_y(vertex.y);
        point.set_z(vertex.z);

        auto it = std::find_if(line.path().begin(), line.path().end(), [&point](const navit::protocol::map_info::Point &p) {
            return p.x() == point.x() && p.y() == point.y();
        });

        if (it == line.path().end()) {
          line.add_path()->CopyFrom(point);
        }
      }
      map_info.add_map_lines()->CopyFrom(line);
    }

    for (const TypedPoint &typedPoint : typed_points_) {
      navit::protocol::map_info::MapPoint point;
      point = typedPoint.propertyType;

      navit::protocol::map_info::Point point_;
      point_.set_x(typedPoint.vertex.x);
      point_.set_y(typedPoint.vertex.y);
      point_.set_z(typedPoint.vertex.z);
      point_.set_rx(typedPoint.euler.x);
      point_.set_ry(typedPoint.euler.y);
      point_.set_rz(typedPoint.euler.z);
      point.mutable_point()->CopyFrom(point_);
      map_info.add_map_points()->CopyFrom(point);
    }
    return true;
  }
  bool CombinedSelectionTool::clearMap() {
    polygons_.clear();
    lines_.clear();
    points_.clear();

    typed_lines_.clear();
    typed_points_.clear();
    vertices_.clear();

    select_polygons_.clear();
    select_lines_.clear();
    select_points_.clear();

    clearHighlights();
    updateVisual();
    return true;
  }
  bool CombinedSelectionTool::isPolygonSelected(const TypedPolygon& polygon, const rviz::ViewportMouseEvent& event)
  {
    Ogre::Camera* camera = context_->getViewManager()->getCurrent()->getCamera();
    Ogre::Ray mouseRay = camera->getCameraToViewportRay(static_cast<float>(event.x) / event.viewport->getActualWidth(),
                                                        static_cast<float>(event.y) / event.viewport->getActualHeight());
    Ogre::Plane groundPlane(Ogre::Vector3::UNIT_Z, 0.0);

    std::pair<bool, Ogre::Real> intersection = mouseRay.intersects(groundPlane);
    if (intersection.first) {
      Ogre::Vector3 mousePosition = mouseRay.getPoint(intersection.second);

      int count = 0;
      size_t n = polygon.vertices.size();
      for (size_t i = 0; i < n; i++) {
        Ogre::Vector3 a = polygon.vertices[i];
        Ogre::Vector3 b = polygon.vertices[(i + 1) % n];
        if ((a.y <= mousePosition.y && mousePosition.y < b.y) || (b.y <= mousePosition.y && mousePosition.y < a.y)) {
          Ogre::Real t = (mousePosition.y - a.y) / (b.y - a.y);
          if (mousePosition.x < a.x + t * (b.x - a.x)) {
            count++;
          }
        }
      }
      return count % 2 != 0;
    }
    return false;
  }

void CombinedSelectionTool::highlightPolygon(const TypedPolygon& polygon, int index)
{
  std::string materialName = "HighlightPolygonMaterial";

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(materialName);
  if (material.isNull()) {
    material = Ogre::MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setDiffuse(Ogre::ColourValue(1, 1, 0, 1));
    material->setAmbient(Ogre::ColourValue(1, 1, 0, 1));
    material->setLightingEnabled(false);
  }
  Ogre::ManualObject* manual = context_->getSceneManager()->createManualObject();
  manual->begin(materialName, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  Ogre::Camera* camera = context_->getViewManager()->getCurrent()->getCamera();
  Ogre::Vector3 cameraPosition = camera->getPosition();

  Ogre::Viewport* viewport = camera->getViewport();

  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();

  float thickness = 0.15;

  for (size_t i = 0; i < polygon.vertices.size(); i++) {
    Ogre::Vector3 start = polygon.vertices[i];
    Ogre::Vector3 end = polygon.vertices[(i + 1) % polygon.vertices.size()];

    Ogre::Vector3 direction = (end - start).normalisedCopy();
    Ogre::Vector3 offset = Ogre::Vector3(-direction.y, direction.x, 0) * thickness / 2.0f;

    Ogre::Vector3 vertices[4];
    vertices[0] = start + offset;
    vertices[1] = start - offset;
    vertices[2] = end - offset;
    vertices[3] = end + offset;

    int baseIdx = i * 4;
    manual->position(vertices[0]);
    manual->position(vertices[1]);
    manual->position(vertices[2]);
    manual->position(vertices[3]);

    manual->quad(baseIdx, baseIdx + 1, baseIdx + 2, baseIdx + 3);
  }

  manual->end();

  context_->getSceneManager()->getRootSceneNode()->attachObject(manual);
  highlightedPolygonObjects_[index] = manual;
}

  void CombinedSelectionTool::clearHighlights()
  {
    for (auto& pair : highlightedPolygonObjects_) {
        Ogre::ManualObject* obj = pair.second;
        if (obj != nullptr) {
          context_->getSceneManager()->getRootSceneNode()->detachObject(obj);
          context_->getSceneManager()->destroyManualObject(obj);
        }
      }
     highlightedPolygonObjects_.clear();
  }

  void CombinedSelectionTool::unhighlightPolygon(int index)
  {
    auto it = highlightedPolygonObjects_.find(index);
    if (it != highlightedPolygonObjects_.end() && it->second != nullptr) {
      Ogre::ManualObject* manual = it->second;
      context_->getSceneManager()->destroyManualObject(manual);
      highlightedPolygonObjects_.erase(it);
    }

    auto it_polygons = select_polygons_.find(index);

    if (it_polygons != select_polygons_.end()) {
      select_polygons_.erase(it_polygons);
    }
  }

  void CombinedSelectionTool::toggleHighlightPolygon(const TypedPolygon& polygon, int index)
  {
    if (highlightedPolygonIndices_.find(index) != highlightedPolygonIndices_.end()) {
      unhighlightPolygon(index);
      highlightedPolygonIndices_.erase(index);
    } else {
      highlightPolygon(polygon, index);
      highlightedPolygonIndices_.insert(index);
    }
  }

  void CombinedSelectionTool::toggleHighlightLine(const Ogre::Vector3& start, const Ogre::Vector3& end, int index)
  {
    if (highlightedLineIndices_.find(index) != highlightedLineIndices_.end()) {
      unhighlightLineSegment(index);
      highlightedLineIndices_.erase(index);
    } else {
      highlightLineSegment(start, end, index);
      highlightedLineIndices_.insert(index);
    }
  }

  bool CombinedSelectionTool::isLineSegmentSelected(const Ogre::Vector3& start, const Ogre::Vector3& end, const Ogre::Vector3& event) {
    float distanceThreshold = 0.2f;

    Ogre::Vector2 mousePoint(event.x, event.y);
    Ogre::Vector2 lineStart(start.x, start.y);
    Ogre::Vector2 lineEnd(end.x, end.y);

    float distance = distanceToLineSegment(mousePoint, lineStart, lineEnd);
    return distance < distanceThreshold;
  }

void CombinedSelectionTool::highlightLineSegment(const Ogre::Vector3& start, const Ogre::Vector3& end, int index) {
  static const std::string materialName = "HighlightLineMaterial";

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName("HighlightPointMaterial");

  if (material.isNull()) {
    material = Ogre::MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setDiffuse(Ogre::ColourValue(1, 1, 0, 1));
    material->setAmbient(Ogre::ColourValue(1, 1, 0, 1));
    material->setLightingEnabled(false);
  }

  Ogre::ManualObject* manual = createLineSegment(start ,end ,0.15f, materialName);

  context_->getSceneManager()->getRootSceneNode()->attachObject(manual);
  highlightedLineObjects_[index] = manual;
}

void CombinedSelectionTool::unhighlightLineSegment(int index) {
  auto it = highlightedLineObjects_.find(index);
  if (it != highlightedLineObjects_.end() && it->second != nullptr) {
    Ogre::ManualObject* manual = it->second;
    context_->getSceneManager()->destroyManualObject(manual);
    highlightedLineObjects_.erase(it);
  }

  auto it_polygons = select_lines_.find(index);

  if (it_polygons != select_lines_.end()) {
    select_lines_.erase(it_polygons);
  }
}

void CombinedSelectionTool::unhighlightPoint(int index) {
    auto it = highlightedPointObjects_.find(index);
    if (it != highlightedPointObjects_.end()) {
        context_->getSceneManager()->destroyManualObject(it->second);
        highlightedPointObjects_.erase(it);
    }
}

void CombinedSelectionTool::toggleHighlightPoint(const Ogre::Vector3& point, int index) {
    if (highlightedPointObjects_.find(index) != highlightedPointObjects_.end()) {
      unhighlightPoint(index);
    } else {
      highlightPoint(point, index);
    }
}

void CombinedSelectionTool::clearLineHighlights() {
  for (auto& pair : highlightedLineObjects_) {
    Ogre::ManualObject* obj = pair.second;
    if (obj != nullptr) {
      context_->getSceneManager()->getRootSceneNode()->detachObject(obj);
      context_->getSceneManager()->destroyManualObject(obj);
    }
  }
  highlightedLineObjects_.clear();
}

  float CombinedSelectionTool::distanceToLineSegment(const Ogre::Vector2& point, const Ogre::Vector2& lineStart, const Ogre::Vector2& lineEnd)
  {
    Ogre::Vector2 lineDiff = lineEnd - lineStart;
    Ogre::Vector2 pointDiff = point - lineStart;

    float t = pointDiff.dotProduct(lineDiff) / lineDiff.squaredLength();
    t = std::max(0.0f, std::min(1.0f, t));

    Ogre::Vector2 closestPoint = lineStart + lineDiff * t;

    return (point - closestPoint).length();
  }

  void CombinedSelectionTool::highlightPoint(const Ogre::Vector3& point, int index) {
    static const std::string materialName = "HighlightPointMaterial";
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(materialName);
    if (material.isNull()) {
      material = Ogre::MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      material->setDiffuse(Ogre::ColourValue(1, 0, 0, 1));
      material->setAmbient(Ogre::ColourValue(1, 0, 0, 1));
      material->getTechnique(0)->setLightingEnabled(false);
    }

    Ogre::ManualObject* manual = createHighlightPoint(point, 0.3, materialName);
    context_->getSceneManager()->getRootSceneNode()->attachObject(manual);
    highlightedPointObjects_[index] = manual;
  }

  bool CombinedSelectionTool::isPointSelected(const Ogre::Vector3& point, const Ogre::Vector3& event) {
      const float selectionThreshold = 0.3f;
      float distance = (point - event).length();
      return distance < selectionThreshold;
  }

  bool CombinedSelectionTool::getSelectArea(std::vector<std::string>& polygons_id) {
    polygons_id.clear();
    polygons_id.reserve(select_polygons_.size());

    for (const auto& entry : select_polygons_) {
      const TypedPolygon& polygon = entry.second; // 获取多边形，从键值对中
      polygons_id.push_back(polygon.propertyType.name());
    }

    draw_type_ = DrawType::NONE;
    return true;
  }

  bool CombinedSelectionTool::getSelectLine(std::vector<std::string>& lines) {
    lines.clear();
    lines.reserve(select_lines_.size());

    for (const auto& entry : select_lines_) {
      const TypedLine& line = entry.second;
      lines.push_back(line.propertyType.name());
    }

    draw_type_ = DrawType::NONE;
    return true;
  }

  bool CombinedSelectionTool::getSelectPoint(std::vector<navit::protocol::map_info::Point>& points) {

    points.clear();

    for (const auto& entry : select_points_) {
      const TypedPoint& select_point = entry.second;
      navit::protocol::map_info::Point point;;
      point.set_x(select_point.vertex.x);
      point.set_y(select_point.vertex.y);
      point.set_z(select_point.vertex.z);

      point.set_rx(0.0);
      point.set_ry(0.0);
      point.set_rz(0.0);
      points.push_back(point);
    }

    draw_type_ = DrawType::NONE;
    return true;
  }

  Ogre::ManualObject* CombinedSelectionTool::createLineSegment(const Ogre::Vector3& start, const Ogre::Vector3& end, float thickness, const std::string& materialName) {
    Ogre::ManualObject* manual = context_->getSceneManager()->createManualObject();
    manual->begin(materialName, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    Ogre::Vector3 direction = (end - start).normalisedCopy();
    Ogre::Vector3 offset = Ogre::Vector3(-direction.y, direction.x, 0) * thickness / 2.0f;

    Ogre::Vector3 vertices[8];
    vertices[0] = start + offset;
    vertices[1] = start - offset;
    vertices[2] = end - offset;
    vertices[3] = end + offset;
    vertices[4] = start + offset + direction * thickness;
    vertices[5] = start - offset + direction * thickness;
    vertices[6] = end - offset + direction * thickness;
    vertices[7] = end + offset + direction * thickness;

    for (int i = 0; i < 8; ++i) {
      manual->position(vertices[i]);
    }

    // Connect the vertices to form the cuboid
    int indices[36] = {0, 1, 2, 2, 3, 0, 3, 2, 6, 6, 7, 3, 7, 6, 5, 5, 4, 7, 4, 5, 1, 1, 0, 4, 1, 5, 6, 6, 2, 1, 4, 0, 3, 3, 7, 4};

    for (int i = 0; i < 36; i += 3) {
      manual->triangle(indices[i], indices[i + 1], indices[i + 2]);
    }

    manual->end();

    return manual;
  }
  Ogre::ManualObject* CombinedSelectionTool::createGeometry(const std::vector<std::pair<Ogre::Vector3, Ogre::Vector3>>& segments, float thickness, const std::string& materialName) {
    Ogre::ManualObject* manual = context_->getSceneManager()->createManualObject();
    manual->begin(materialName, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    for (const auto& segment : segments) {
      Ogre::Vector3 start = segment.first;
      Ogre::Vector3 end = segment.second;
      Ogre::Vector3 direction = (end - start).normalisedCopy();
      Ogre::Vector3 offset = Ogre::Vector3(-direction.y, direction.x, 0) * thickness / 2.0f;

      manual->position(start + offset);
      manual->position(start - offset);
      manual->position(end - offset);
      manual->position(end + offset);

      // Connect the vertices to form a rectangle
      int baseIdx = segments.size() * 4;
      manual->quad(baseIdx, baseIdx + 1, baseIdx + 2, baseIdx + 3);
    }

    manual->end();
    return manual;
  }
  Ogre::ManualObject* CombinedSelectionTool::createHighlightPoint(const Ogre::Vector3& point, float size, const std::string& materialName) {
    Ogre::ManualObject* manual = context_->getSceneManager()->createManualObject();
    manual->begin(materialName, Ogre::RenderOperation::OT_TRIANGLE_FAN);
    manual->position(point);

    std::vector<Ogre::Vector3> vertices = calculateStarVertices(point, size);

    for (const Ogre::Vector3& vertex : vertices) {
      manual->position(vertex);
    }

    manual->end();
    return manual;
  }
  std::vector<Ogre::Vector3> CombinedSelectionTool::calculateStarVertices(const Ogre::Vector3& center, float size) {
    std::vector<Ogre::Vector3> vertices;
    float angleStep = 2 * Ogre::Math::PI / 5;
    float sinPIOver10 = std::sin(Ogre::Math::PI / 10);
    float sin7PIOver10 = std::sin(7 * Ogre::Math::PI / 10);
    float innerRadius = size * sinPIOver10 / sin7PIOver10;

    for (int i = 0; i < 5; i++) {
      float outerAngle = i * angleStep;
      float innerAngle = outerAngle + angleStep / 2;

      Ogre::Vector3 outerPoint = center + Ogre::Vector3(std::cos(outerAngle), std::sin(outerAngle), 0) * size;
      Ogre::Vector3 innerPoint = center + Ogre::Vector3(std::cos(innerAngle), std::sin(innerAngle), 0) * innerRadius;

      vertices.push_back(outerPoint);
      vertices.push_back(innerPoint);
    }

    return vertices;
  }
  void CombinedSelectionTool::clearPointHighlights() {
    for (auto& pair : highlightedPointObjects_) {
      Ogre::ManualObject* obj = pair.second;
      if (obj != nullptr) {
        context_->getSceneManager()->getRootSceneNode()->detachObject(obj);
        context_->getSceneManager()->destroyManualObject(obj);
      }
    }
    highlightedPointObjects_.clear();
  }
  void CombinedSelectionTool::unhighlightAll() {
    clearHighlights();
    clearLineHighlights();
    clearPointHighlights();
  }
void CombinedSelectionTool::createText(const std::string& text, const Ogre::Vector3& position, const std::string& id) {
    static int idCounter = 0;
    std::string baseId = "DynamicText_" + std::to_string(idCounter++); // 使用 idCounter 增量以确保唯一性
    std::string panelId = baseId + "_Panel";
    std::string textAreaId = baseId + "_TextArea";
    std::string overlayId = baseId + "_Overlay";

    Ogre::OverlayManager& overlayManager = Ogre::OverlayManager::getSingleton();

    Ogre::TextAreaOverlayElement* textArea = static_cast<Ogre::TextAreaOverlayElement*>(
        overlayManager.createOverlayElement("TextArea", textAreaId));
    if (!textArea) {
        ROS_ERROR("Could not create text area.");
        return;
    }
    textArea->setMetricsMode(Ogre::GMM_PIXELS);
    textArea->setPosition(0, 0);
    textArea->setDimensions(100, 100);
    textArea->setCaption(text);
    textArea->setCharHeight(16);
    textArea->setFontName("Liberation Sans");
    textArea->setColourBottom(Ogre::ColourValue::White); // 文本颜色
    textArea->setColourTop(Ogre::ColourValue::White);// 文本颜色

    Ogre::OverlayContainer* panel = static_cast<Ogre::OverlayContainer*>(
        overlayManager.createOverlayElement("Panel", panelId));
    if (!panel) {
        ROS_ERROR("Could not create panel.");
        return;
    }
    panel->setMetricsMode(Ogre::GMM_PIXELS);
    Ogre::Camera* camera = context_->getViewManager()->getCurrent()->getCamera();

    Ogre::Vector2 screenPosition = convertWorldToScreenPosition(position, camera);
    panel->setPosition(screenPosition.x, screenPosition.y);
    panel->setDimensions(1, 1);
    panel->addChild(textArea);

    Ogre::Overlay* overlay = overlayManager.create(overlayId);
    if (!overlay) {
        ROS_ERROR("Could not create overlay.");
        return;
    }
    overlay->add2D(panel);
    overlay->setZOrder(500);
    overlay->show();
  }

  void CombinedSelectionTool::addLineSegment(const Ogre::Vector3& start, const Ogre::Vector3& end, LineType type) {
    switch (type) {
      case LINE_TYPE_NORMAL:
        // normal_lines_.push_back(start);
        // normal_lines_.push_back(end);
        break;
      case LINE_TYPE_TEACHING:
        if (teaching_points_.empty() || teaching_points_.back() != start) {
          teaching_points_.push_back(start);
        }
        teaching_points_.push_back(end);
        break;
      default:
        break;
    }
  }
  Ogre::Vector2 CombinedSelectionTool::convertWorldToScreenPosition(const Ogre::Vector3& worldPos, Ogre::Camera* camera) {
  if (!camera) return Ogre::Vector2::ZERO;
  Ogre::Vector3 screenPos = camera->getProjectionMatrix() * (camera->getViewMatrix() * worldPos);
  return Ogre::Vector2(
      (screenPos.x / 2 + 0.5) * camera->getViewport()->getActualWidth(),
      (1 - (screenPos.y / 2 + 0.5)) * camera->getViewport()->getActualHeight()
  );
  }

  void CombinedSelectionTool::publishShapes(const std::vector<TypedPoint>& typed_points,
                    const std::vector<TypedLine>& typed_lines,
                    const std::vector<TypedPolygon>& typed_polygons,
                    const ros::Publisher& points_marker_pub,
                    const ros::Publisher& lines_marker_pub,
                    const ros::Publisher& polygons_marker_pub,
                    const std::string& frame_id) {

    visualization_msgs::MarkerArray names_marker_array;
    uint32_t names_id = 0;

    clearAllMarkers(points_marker_pub, "points");
    clearAllMarkers(lines_marker_pub, "lines");
    clearAllMarkers(polygons_marker_pub, "polygons");

    visualization_msgs::MarkerArray points_array;
    visualization_msgs::MarkerArray lines_array;
    visualization_msgs::MarkerArray polygons_array;

    // Iterate over typed_points and create point markers
    for (const auto& typed_point : typed_points) {
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = frame_id;
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "points";
        point_marker.id = typed_point.propertyType.id();
        point_marker.text = typed_point.propertyType.name();
        point_marker.type = visualization_msgs::Marker::POINTS;
        point_marker.action = visualization_msgs::Marker::ADD;
        point_marker.pose.orientation.w = 1.0;
        point_marker.scale.x = 0.1;  // Point size
        point_marker.scale.y = 0.1;  // Point size
        point_marker.color.r = 1.0;  // Red color
        point_marker.color.a = 1.0;  // Alpha (transparency)

        // Convert TypedPoint to geometry_msgs::Point and add to marker
        point_marker.points.push_back(convertPointToGeomPoint(typed_point));

        points_array.markers.push_back(point_marker);

        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "text";
        text_marker.id = names_id;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = typed_point.vertex.x + 0.1;
        text_marker.pose.position.y = typed_point.vertex.y + 0.1;
        text_marker.scale.z = 0.5;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = typed_point.propertyType.name();
        names_id++;
        names_marker_array.markers.push_back(text_marker);
    }

    // Iterate over typed_lines and create line markers
    for (const auto& typed_line : typed_lines) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = frame_id;
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "lines";
        line_marker.id = typed_line.propertyType.id();
        line_marker.text = typed_line.propertyType.name();
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = 0.05;  // Line width
        line_marker.color.g = 1.0;  // Green color
        line_marker.color.a = 1.0;  // Alpha (transparency)

        // Convert TypedLine to a vector of geometry_msgs::Point and add to marker
        line_marker.points = convertLineToPoints(typed_line);

        lines_array.markers.push_back(line_marker);

        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "text";
        text_marker.id = names_id;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = typed_line.vertices[0].x + 0.1;
        text_marker.pose.position.y = typed_line.vertices[0].y + 0.1;
        text_marker.scale.z = 0.5;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = typed_line.propertyType.name();
        names_id++;
        names_marker_array.markers.push_back(text_marker);
    }

    // Iterate over typed_polygons and create polygon markers
    for (const auto& typed_polygon : typed_polygons) {
        visualization_msgs::Marker polygon_marker;
        polygon_marker.header.frame_id = frame_id;
        polygon_marker.header.stamp = ros::Time::now();
        polygon_marker.ns = "polygons";
        polygon_marker.id = typed_polygon.propertyType.id();
        polygon_marker.text = typed_polygon.propertyType.name();
        polygon_marker.type = visualization_msgs::Marker::LINE_STRIP;  // LINE_STRIP for an open polygon, LINE_LOOP for a closed polygon
        polygon_marker.action = visualization_msgs::Marker::ADD;
        polygon_marker.pose.orientation.w = 1.0;
        polygon_marker.scale.x = 0.05;  // Line width
        polygon_marker.color.b = 1.0;  // Blue color
        polygon_marker.color.a = 1.0;  // Alpha (transparency)

        // Convert TypedPolygon to a vector of geometry_msgs::Point and add to marker
        polygon_marker.points = convertPolygonToPoints(typed_polygon);

        polygons_array.markers.push_back(polygon_marker);

        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "text";
        text_marker.id = names_id;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = typed_polygon.vertices[0].x + 0.1;
        text_marker.pose.position.y = typed_polygon.vertices[0].y + 0.1;
        text_marker.scale.z = 0.5;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = typed_polygon.propertyType.name();
        names_id++;
        names_marker_array.markers.push_back(text_marker);
    }

    // Now publish the MarkerArray messages
    points_marker_pub.publish(points_array);
    lines_marker_pub.publish(lines_array);
    polygons_marker_pub.publish(polygons_array);
    names_marker_pub_.publish(names_marker_array);

  }
}  // namespace rviz_combined_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_combined_tool::CombinedSelectionTool, rviz::Tool)