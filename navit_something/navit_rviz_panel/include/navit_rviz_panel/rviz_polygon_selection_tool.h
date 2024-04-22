#include <navit_rviz_panel/GetSelection.h>
#include <geometry_msgs/PolygonStamped.h>
#include <navit_rviz_panel/PolygonArrayStamped.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <rviz/tool.h>
#include <ros/ros.h>

namespace rviz
{
class BoolProperty;
class ColorProperty;
class FloatProperty;
}  // namespace rviz

class OgreManualObject;

namespace rviz_polygon_selection_tool
{
class PolygonSelectionTool : public rviz::Tool
{
  Q_OBJECT

public:
  PolygonSelectionTool();

  void onInitialize() override;
  void activate() override {};
  void deactivate() override {};
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

public Q_SLOTS:
  void updatePtsColor();
  void updatePtsSize();
  void updateLinesColor();
  void updateVisual();

private:
  bool callback(navit_rviz_panel::GetSelection::Request& req, navit_rviz_panel::GetSelection::Response& res);

  rviz::BoolProperty* lasso_mode_property_;
  rviz::BoolProperty* close_loop_property_;
  rviz::ColorProperty* pt_color_property_;
  rviz::ColorProperty* line_color_property_;
  rviz::FloatProperty* pt_size_property_;

  ros::ServiceServer server_;

  // std::vector<Ogre::Vector3> points_;
  // std::vector<std::vector<Ogre::Vector3>> polygons_;
  Ogre::ManualObject* pts_vis_;
  Ogre::ManualObject* lines_vis_;
  Ogre::MaterialPtr pts_material_;
  Ogre::MaterialPtr lines_material_;

  Ogre::MaterialPtr red_material_;
  Ogre::MaterialPtr yellow_material_;
  std::vector<std::vector<std::vector<Ogre::Vector3>>> polygons_;
};

}  // namespace rviz_polygon_selection_tool
