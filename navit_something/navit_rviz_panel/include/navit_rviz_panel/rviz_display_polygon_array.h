#ifndef POLYGON_ARRAY_DISPLAY_H
#define POLYGON_ARRAY_DISPLAY_H

#include <rviz/message_filter_display.h>
#include <navit_rviz_panel/PolygonArray.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
namespace polygon_array_display
{

class PolygonArrayDisplay: public rviz::MessageFilterDisplay<navit_msgs::PolygonArray>
{
Q_OBJECT

public:
  PolygonArrayDisplay();
  virtual ~PolygonArrayDisplay();

protected:
  virtual void onInitialize();

  virtual void processMessage( const navit_msgs::PolygonArray::ConstPtr& msg );

private:
  //rviz::ColorProperty* color_property_;
  std::vector<Ogre::ManualObject*> polygons_;
};

} // end namespace polygon_array_display

#endif // POLYGON_ARRAY_DISPLAY_H