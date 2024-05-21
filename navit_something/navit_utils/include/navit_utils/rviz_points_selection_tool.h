#include <navit_utils/GetPoints.h>
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

namespace rviz_point_selection_tool
{
class PointsSelectionTool : public rviz::Tool
{
  Q_OBJECT

public:
  PointsSelectionTool();

  void onInitialize() override;
  void activate() override {};
  void deactivate() override {};
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  
public Q_SLOTS:
  void updatePtsSize();
  void updateVisual();

private:
  bool callback(navit_utils::GetPoints::Request& req, navit_utils::GetPoints::Response& res);

  rviz::FloatProperty* pt_size_property_;

  ros::ServiceServer server_;

  Ogre::ManualObject* pts_vis_;
  Ogre::MaterialPtr pts_material_;
  std::vector<Ogre::Vector3> points_;
};

}  // namespace rviz_point_selection_tool
