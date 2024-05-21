#include <navit_utils/rviz_points_selection_tool.h>

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

namespace rviz_point_selection_tool
{
PointsSelectionTool::PointsSelectionTool() 
{
  shortcut_key_ = 'l';
}

void PointsSelectionTool::onInitialize()
{
  ros::NodeHandle nh;
  
  server_ = nh.advertiseService("get_work_points", &PointsSelectionTool::callback, this);

  // Add the points visualization
  pts_vis_ = scene_manager_->createManualObject("work_points");

  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(pts_vis_);

  pts_material_ = Ogre::MaterialManager::getSingleton().create("points_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  pt_size_property_ = new rviz::FloatProperty("Point Size", 5.0, "Size of clicked points",
                                                                 getPropertyContainer(), SLOT(updatePtsSize()), this);

  // Update the materials
  updatePtsSize();
}

int PointsSelectionTool::processMouseEvent(rviz::ViewportMouseEvent& event)
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

void PointsSelectionTool::updatePtsSize()
{
  pts_material_->setPointSize(pt_size_property_->getFloat());
}

bool PointsSelectionTool::callback(navit_utils::GetPoints::Request& req, navit_utils::GetPoints::Response& res)
{ 
  for (const Ogre::Vector3& pt : points_)
  {
    geometry_msgs::Point32 point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    res.points.push_back(point);
  }

  return true;
}

void PointsSelectionTool::updateVisual()
{
  // Clear the visual
  pts_vis_->clear();
  pts_vis_->estimateVertexCount(points_.size());

  // Set the material
  pts_vis_->begin(pts_material_->getName(), Ogre::RenderOperation::OT_POINT_LIST);

  for (const Ogre::Vector3& pt : points_)
  {
    pts_vis_->position(pt);
    pts_vis_->colour(1.0f, 0.0f, 0.0f, 1.0f);
  }

  pts_vis_->end();
}
}  // namespace rviz_point_selection_tool
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_point_selection_tool::PointsSelectionTool, rviz::Tool)
