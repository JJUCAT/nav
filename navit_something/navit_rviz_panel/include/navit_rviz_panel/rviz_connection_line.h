#include <navit_rviz_panel/GetPolylines.h>
#include <geometry_msgs/Point32.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <rviz/tool.h>
#include <ros/ros.h>

namespace rviz
{
class FloatProperty;
}  // namespace rviz

class OgreManualObject;

namespace rviz_line_segment_tool
{
class LineSegmentTool : public rviz::Tool
{
  Q_OBJECT

public:
  LineSegmentTool();

  void onInitialize() override;
  void activate() override {};
  void deactivate() override {};
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  
public Q_SLOTS:
  void updatePtsSize();
  void updateVisual();

private:
  bool callback(navit_rviz_panel::GetPolylines::Request& req, navit_rviz_panel::GetPolylines::Response& res);

  rviz::FloatProperty* pt_size_property_;

  ros::ServiceServer server_;

  Ogre::ManualObject* pts_vis_;
  Ogre::MaterialPtr pts_material_;
  std::vector<Ogre::Vector3> points_;
};

}  // namespace rviz_line_segment_tool
