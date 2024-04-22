#include "navit_rviz_panel/rviz_display_polygon_array.h"

#include <rviz/validate_floats.h>
#include <rviz/properties/color_property.h>

namespace polygon_array_display
{

PolygonArrayDisplay::PolygonArrayDisplay()
{
  //color_property_ = new rviz::ColorProperty( "Color", QColor( 255, 255, 0 ), "Color to draw the polygons.", this );
}

PolygonArrayDisplay::~PolygonArrayDisplay()
{
}

void PolygonArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void PolygonArrayDisplay::processMessage( const navit_msgs::PolygonArray::ConstPtr& msg )
{    // Clear old polygons
    for (size_t i = 0; i < polygons_.size(); ++i) {
        context_->getSceneManager()->destroyManualObject(polygons_[i]);
    }
    polygons_.clear();

    // Add new polygons
    for (size_t i = 0; i < msg->polygon_array.size(); ++i) {
        const auto& polygon_msg = msg->polygon_array[i];

        // Ignore invalid polygons
        if (polygon_msg.points.size() < 3) {
            ROS_WARN_STREAM("Ignoring invalid polygon with less than 3 points.");
            continue;
        }

        // Create a new manual object
        std::ostringstream name;
        name << "Polygon_" << i;
        Ogre::ManualObject* manual = scene_manager_->createManualObject(name.str());
        manual->setDynamic(false);

        // Begin drawing lines
        manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

        // Add vertices
        for (const auto& point : polygon_msg.points) {
            manual->position(point.x, point.y, point.z);
        }

        // Close the polygon by connecting the last vertex with the first
        const auto& first_point = polygon_msg.points[0];
        manual->position(first_point.x, first_point.y, first_point.z);

        // End drawing
        manual->end();

        // Add the object to the scene
        scene_node_->attachObject(manual);

        // Save the object for later deletion
        polygons_.push_back(manual);
    }

} // end namespace polygon_array_display
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(polygon_array_display::PolygonArrayDisplay, rviz::Display )