#ifndef NAVIT_COMMON__COORDINATE_TRANS_H
#define NAVIT_COMMON__COORDINATE_TRANS_H

#include <ros/ros.h>
#include <tinyxml2.h>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

#include <vector>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <navit_common/log.h>

namespace navit_common {
namespace coordinate_trans {
    struct NamedUTMPoint {
        enum TYPE {
            POINT,
            LINE,
            POLYGON
        };

        std::vector<geodesy::UTMPoint> utm_points;
        std::string name;
        TYPE type;
    };
    void parseKMLAndConvertToUtm(const std::string& file_path, std::vector<NamedUTMPoint>& named_utm_points);
    void convertUtmToRosPath(const std::vector<geodesy::UTMPoint>& utm_points, nav_msgs::Path& path);
    void utm2map(std::vector<NamedUTMPoint>& named_utm_points);
    void map2utm(std::vector<NamedUTMPoint>& named_utm_points);
    void convertLocalToKml(const std::vector<NamedUTMPoint>& local_points, const std::string file_path);
} //coordinate_trans
} //navit_common
#endif