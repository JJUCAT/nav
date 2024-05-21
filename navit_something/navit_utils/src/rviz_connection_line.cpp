#include <navit_utils/rviz_connection_line.h>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/properties/float_property.h>

static void updateMaterialColor(Ogre::MaterialPtr material, const QColor& color)
{
  qreal r, g, b, a;
  color.getRgbF(&r, &g, &b, &a);
  material->setDiffuse(r, g, b, a);
  material->setSpecular(r, g, b, a);
  material->setAmbient(r, g, b);
}

namespace rviz_line_segment_tool
{
LineSegmentTool::LineSegmentTool() 
{
  shortcut_key_ = 'l';
}

void LineSegmentTool::onInitialize()
{
  ros::NodeHandle nh;
  
  server_ = nh.advertiseService("get_connection_line", &LineSegmentTool::callback, this);

  // Add the points visualization
  pts_vis_ = scene_manager_->createManualObject("connection_line");

  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(pts_vis_);

  pts_material_ = Ogre::MaterialManager::getSingleton().create("points_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  pt_size_property_ = new rviz::FloatProperty("Point Size", 5.0, "Size of clicked points",
                                                                 getPropertyContainer(), SLOT(updatePtsSize()), this);

  // Update the materials
  updatePtsSize();
}

int LineSegmentTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  // Collect the point
  if (event.leftUp())
  {
    Ogre::Camera* camera = context_->getViewManager()->getCurrent()->getCamera();
    Ogre::Ray mouseRay = camera->getCameraToViewportRay(static_cast<float>(event.x) / event.viewport->getActualWidth(),
                                                        static_cast<float>(event.y) / event.viewport->getActualHeight());
    Ogre::Vector3 position;
    // perform a raycast or similar to find the 3D point
    Ogre::Plane groundPlane(Ogre::Vector3::UNIT_Z, 0.0);

    std::pair<bool, Ogre::Real> intersection = mouseRay.intersects(groundPlane);
     if (intersection.first) // if the ray intersects the plane
    {
      Ogre::Vector3 position = mouseRay.getPoint(intersection.second); // get the intersection point
      points_.push_back(position);
      updateVisual();
    }
  }
  else if (event.right())
  {
    points_.clear();
    pts_vis_->clear();
  }

  return Render;
}

void LineSegmentTool::updatePtsSize()
{
  pts_material_->setPointSize(pt_size_property_->getFloat());
}

bool LineSegmentTool::callback(navit_utils::GetPolylines::Request& req, navit_utils::GetPolylines::Response& res)
{ 
  navit_utils::Polyline polyline;  // Assuming Polyline is a struct containing a vector of geometry_msgs::Point
  for (size_t i = 0; i < points_.size(); ++i)
  {
    geometry_msgs::Point point;
    point.x = points_[i].x;
    point.y = points_[i].y;
    point.z = points_[i].z;
    polyline.polyline.push_back(point);

    if (i % 2 != 0) // if i is odd, which means we already have 2 points in polyline
    {
      res.polylines.push_back(polyline);
      polyline.polyline.clear(); // clear the points for next polyline
    }
  }

  if (!polyline.polyline.empty())
  {
    res.polylines.push_back(polyline);
  }
  return true;
}

void LineSegmentTool::updateVisual()
{
  // Clear the visual
  pts_vis_->clear();

  if (points_.size() < 2) // we need at least 2 points to create a line
    return;

  for (size_t i = 0; i < points_.size(); i += 2)
  {
    // Set the material
    pts_vis_->begin(pts_material_->getName(), Ogre::RenderOperation::OT_LINE_STRIP);

    for (size_t j = i; j < std::min(i + 2, points_.size()); ++j)
    {
      pts_vis_->position(points_[j]);
      pts_vis_->colour(1.0f, 0.0f, 0.0f, 1.0f); // set line color as needed
    }

    pts_vis_->end();
  }
}
}  // namespace rviz_line_segment_tool
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_line_segment_tool::LineSegmentTool, rviz::Tool)
