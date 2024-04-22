#include "navit_rviz_panel/rviz_polygon_selection_tool.h"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

static void updateMaterialColor(Ogre::MaterialPtr material, const QColor& color)
{
  qreal r, g, b, a;
  color.getRgbF(&r, &g, &b, &a);
  material->setDiffuse(r, g, b, a);
  material->setSpecular(r, g, b, a);
  material->setAmbient(r, g, b);
}

namespace rviz_polygon_selection_tool
{
PolygonSelectionTool::PolygonSelectionTool() 
{
  shortcut_key_ = 'p';
}

void PolygonSelectionTool::onInitialize()
{
  ros::NodeHandle nh;
  
  server_ = nh.advertiseService("get_selection", &PolygonSelectionTool::callback, this);

  // Add the points visualization
  pts_vis_ = scene_manager_->createManualObject("points");

  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(pts_vis_);

  pts_material_ = Ogre::MaterialManager::getSingleton().create("points_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  // Add the lines visualization
  lines_vis_ = scene_manager_->createManualObject("lines");
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(lines_vis_);
  lines_material_ = Ogre::MaterialManager::getSingleton().create("lines_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  // Add the properties
  lasso_mode_property_ = new rviz::BoolProperty(
      "Lasso mode", true, "Toggle between lasso and discrete click mode", getPropertyContainer());

  close_loop_property_ = new rviz::BoolProperty("Close loop", true,
                                                                   "Close the polygon with a line between the last and "
                                                                   "first points",
                                                                   getPropertyContainer(), SLOT(updateVisual()), this);

  pt_color_property_ = new rviz::ColorProperty("Point Color", Qt::black, "Color of the points",
                                                                  getPropertyContainer(), SLOT(updatePtsColor()), this);

  line_color_property_ = new rviz::ColorProperty(
      "Line Color", Qt::black, "Color of the line", getPropertyContainer(), SLOT(updateLinesColor()), this);

  pt_size_property_ = new rviz::FloatProperty("Point Size", 5.0, "Size of clicked points",
                                                                 getPropertyContainer(), SLOT(updatePtsSize()), this);

  
  red_material_ = Ogre::MaterialManager::getSingleton().create("red_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  red_material_->getTechnique(0)->getPass(0)->setDiffuse(1, 0, 0, 0);
  red_material_->getTechnique(0)->getPass(0)->setAmbient(1, 0, 0);
  red_material_->getTechnique(0)->getPass(0)->setSelfIllumination(1, 0, 0);

  // 创建黄色材质
  yellow_material_ = Ogre::MaterialManager::getSingleton().create("yellow_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  yellow_material_->getTechnique(0)->getPass(0)->setDiffuse(1, 1, 0, 0);
  yellow_material_->getTechnique(0)->getPass(0)->setAmbient(1, 1, 0);
  yellow_material_->getTechnique(0)->getPass(0)->setSelfIllumination(1, 1, 0);
  
  // Update the materials
  updatePtsSize();
  updatePtsColor();
  updateLinesColor();
}

int PolygonSelectionTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  Ogre::Camera* camera = context_->getViewManager()->getCurrent()->getCamera();
  Ogre::Ray mouseRay = camera->getCameraToViewportRay(static_cast<float>(event.x) / event.viewport->getActualWidth(),
                                                      static_cast<float>(event.y) / event.viewport->getActualHeight());
  Ogre::Plane groundPlane(Ogre::Vector3::UNIT_Z, 0.0);

  std::pair<bool, Ogre::Real> intersection = mouseRay.intersects(groundPlane);

  // 左键按下或者拖拽
  if (event.left() || event.leftDown()) {
    if (intersection.first) {
      Ogre::Vector3 position = mouseRay.getPoint(intersection.second);

      if (event.leftDown()) {
        polygons_.push_back(std::vector<std::vector<Ogre::Vector3>>());
        polygons_.back().push_back(std::vector<Ogre::Vector3>());
      }

      polygons_.back()[0].push_back(position);
      updateVisual();
    }
  }

  // 右键按下或者拖拽
  else if (event.right() || event.rightDown()) {
    if (!polygons_.empty() && intersection.first) {
      Ogre::Vector3 position = mouseRay.getPoint(intersection.second);

      if (event.rightDown()) {
        polygons_.back().push_back(std::vector<Ogre::Vector3>());
      }

      polygons_.back().back().push_back(position);
      updateVisual();
    }
  }
  // 中键按下
  else if (event.middleDown()) {
    if (!polygons_.empty())  {
      if (polygons_.back().size() == 1) {
          polygons_.pop_back();
      } else {
        polygons_.back().pop_back();
      }
      updateVisual();
    }
  }

  return Render;
}

void PolygonSelectionTool::updatePtsColor()
{
  return updateMaterialColor(pts_material_, pt_color_property_->getColor());
}

void PolygonSelectionTool::updateLinesColor()
{
  return updateMaterialColor(lines_material_, line_color_property_->getColor());
}

void PolygonSelectionTool::updatePtsSize()
{
  pts_material_->setPointSize(pt_size_property_->getFloat());
}

bool PolygonSelectionTool::callback(navit_rviz_panel::GetSelection::Request& req, navit_rviz_panel::GetSelection::Response& res) { 
  for (const auto& polygonGroup : polygons_) {
    navit_rviz_panel::PolygonArrayStamped polygonArray;
    //polygonArray.header.frame_id = context_->getFixedFrame().toStdString();
    for (std::size_t i = 0; i < polygonGroup.size(); ++i) {
        const auto& polygon = polygonGroup[i];
        if (!polygon.empty()) {
          geometry_msgs::PolygonStamped msg;
          for (const Ogre::Vector3& pt : polygon) {
            geometry_msgs::Point32 point;
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            msg.polygon.points.push_back(point);
          }
          polygonArray.polygons.push_back(msg);
        }
    }

    if (!polygonArray.polygons.empty()) {
      res.selection.push_back(polygonArray);
    }
  }
  return true;
}

void PolygonSelectionTool::updateVisual()
{
    pts_material_->setPointSize(pt_size_property_->getFloat());
    pts_vis_->clear();
    lines_vis_->clear();

    if (!lasso_mode_property_->getBool()) {
        for (const auto& polygonGroup : polygons_) {
            for (const auto& polygon : polygonGroup) {
                if (!polygon.empty()) {
                    pts_vis_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
                    for (const Ogre::Vector3& point : polygon) {
                        pts_vis_->position(point);
                    }
                    pts_vis_->end();
                }
            }
        }
    }
    if (!polygons_.empty()) {
        for (const auto& polygonGroup : polygons_) {
            bool isFirstPolygon = true;
            for (const auto& polygon : polygonGroup) {
                if (polygon.size() > 1) {
                    auto material = isFirstPolygon ? red_material_: yellow_material_;
                    isFirstPolygon = false;

                    lines_vis_->begin(material->getName(), Ogre::RenderOperation::OT_LINE_STRIP);

                    for (std::size_t i = 0; i < polygon.size() - 1; ++i) {
                        lines_vis_->position(polygon.at(i));
                        lines_vis_->position(polygon.at(i + 1));
                    }
                    // Close the polygon
                    if (polygon.size() > 2 && close_loop_property_->getBool()) {
                        lines_vis_->position(polygon.back());
                        lines_vis_->position(polygon.front());
                    }
                    lines_vis_->end();
                }
            }
        }
    }
}

}  // namespace rviz_polygon_selection_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_selection_tool::PolygonSelectionTool, rviz::Tool)
