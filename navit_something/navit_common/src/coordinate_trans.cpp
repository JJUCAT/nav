

#include <navit_common/coordinate_trans.h>

namespace navit_common {
namespace coordinate_trans {
void parseKMLAndConvertToUtm(const std::string& file_path, std::vector<NamedUTMPoint>& named_utm_points) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS) {
        ROS_ERROR("Unable to load KML file.");
        return;
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("kml");
    if (!root) return;

    for (tinyxml2::XMLElement* placemark = root->FirstChildElement("Document")->FirstChildElement("Placemark"); placemark; placemark = placemark->NextSiblingElement("Placemark")) {
        tinyxml2::XMLElement* nameElement = placemark->FirstChildElement("name");
        std::string name;
        if (nameElement) {
            name = nameElement->GetText();
        }

        NamedUTMPoint named_utm_point;
        tinyxml2::XMLElement* point = placemark->FirstChildElement("Point");
        if (point) {
            tinyxml2::XMLElement* coordinates = point->FirstChildElement("coordinates");
            named_utm_point.name = name;
            named_utm_point.type = NamedUTMPoint::POINT;
            if (coordinates) {
                std::string coordText = coordinates->GetText();
                std::replace(coordText.begin(), coordText.end(), ',', ' ');

                std::istringstream iss(coordText);
                double longitude, latitude, altitude;
                iss >> longitude >> latitude >> altitude;

                geographic_msgs::GeoPoint geo_point;
                geo_point.latitude = latitude;
                geo_point.longitude = longitude;

                named_utm_point.utm_points.push_back(geodesy::UTMPoint(geo_point));
             
            }
        }

        tinyxml2::XMLElement* polygon = placemark->FirstChildElement("Polygon");
        if (polygon) {
            tinyxml2::XMLElement* outerBoundary = polygon->FirstChildElement("outerBoundaryIs");
            if (outerBoundary) {
                tinyxml2::XMLElement* linearRing = outerBoundary->FirstChildElement("LinearRing");
                if (linearRing) {
                    tinyxml2::XMLElement* coordinates = linearRing->FirstChildElement("coordinates");
                    if (coordinates) {
                        std::string coordText = coordinates->GetText();
                        std::istringstream iss(coordText);
                        std::string token;
                        named_utm_point.name = name;
                        named_utm_point.type = NamedUTMPoint::POLYGON;

                        while (std::getline(iss, token, ' ')) { 
                            std::istringstream coordStream(token);
                            double longitude, latitude, altitude;
                            char comma;

                            coordStream >> longitude >> comma >> latitude >> comma >> altitude;

                            geographic_msgs::GeoPoint geo_point;
                            geo_point.latitude = latitude;
                            geo_point.longitude = longitude;

                            named_utm_point.utm_points.push_back(geodesy::UTMPoint(geo_point));
                        }

                        while (std::getline(iss, token, ' ')) {
                            std::istringstream coordStream(token);
                            double longitude, latitude, altitude;
                            char comma;

                            coordStream >> longitude >> comma >> latitude >> comma >> altitude;

                            geographic_msgs::GeoPoint geo_point;
                            geo_point.latitude = latitude;
                            geo_point.longitude = longitude;

                            named_utm_point.utm_points.push_back(geodesy::UTMPoint(geo_point));
                        }
                    }
                }
            }
        }

        tinyxml2::XMLElement* line = placemark->FirstChildElement("LineString");
        if (line) {
            tinyxml2::XMLElement* coordinates = line->FirstChildElement("coordinates");
            if (coordinates) {
                std::string coordText = coordinates->GetText();
                std::istringstream iss(coordText);
                std::string token;
                named_utm_point.name = name;
                named_utm_point.type = NamedUTMPoint::LINE;

                while (std::getline(iss, token, ' ')) { 
                    std::istringstream coordStream(token);
                    double longitude, latitude, altitude;
                    char comma;

                    coordStream >> longitude >> comma >> latitude >> comma >> altitude;

                    geographic_msgs::GeoPoint geo_point;
                    geo_point.latitude = latitude;
                    geo_point.longitude = longitude;

                    named_utm_point.utm_points.push_back(geodesy::UTMPoint(geo_point));
                }

                while (std::getline(iss, token, ' ')) {
                    std::istringstream coordStream(token);
                    double longitude, latitude, altitude;
                    char comma;

                    coordStream >> longitude >> comma >> latitude >> comma >> altitude;

                    geographic_msgs::GeoPoint geo_point;
                    geo_point.latitude = latitude;
                    geo_point.longitude = longitude;

                    named_utm_point.utm_points.push_back(geodesy::UTMPoint(geo_point));
                }
            }
        }
        named_utm_points.push_back(named_utm_point);
    }
}
void convertLocalToKml(const std::vector<NamedUTMPoint>& local_points, const std::string file_path) {
    double wgs84_longitude_offset = 113.8646179;
    double wgs84_latitude_offset = 22.9033987;
    geographic_msgs::GeoPoint geo_point_offset;
    geo_point_offset.latitude = wgs84_latitude_offset;
    geo_point_offset.longitude = wgs84_longitude_offset;
    geo_point_offset.altitude = 0.0;
    int utm_zone = (int)((wgs84_longitude_offset + 180.0) / 6) + 1;
    geodesy::UTMPoint utm_point_offset = geodesy::UTMPoint(geo_point_offset);

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement* root = doc.NewElement("kml");
    doc.InsertFirstChild(root);
    tinyxml2::XMLElement* document = doc.NewElement("Document");
    root->InsertEndChild(document);
    for (auto& named_utm_point : local_points) {
        tinyxml2::XMLElement* placemark = doc.NewElement("Placemark");
        document->InsertEndChild(placemark);
        tinyxml2::XMLElement* name = doc.NewElement("name");
        name->SetText(named_utm_point.name.c_str());
        placemark->InsertEndChild(name);
        if (named_utm_point.type == NamedUTMPoint::POINT) {
            tinyxml2::XMLElement* point = doc.NewElement("Point");
            placemark->InsertEndChild(point);
            tinyxml2::XMLElement* coordinates = doc.NewElement("coordinates");
            std::string coordText;
            for (auto& utm_point : named_utm_point.utm_points) {
                geodesy::UTMPoint utm_correct_point;
                utm_correct_point.altitude = utm_point.altitude + utm_point_offset.altitude;
                utm_correct_point.easting = utm_point.easting + utm_point_offset.easting;
                utm_correct_point.northing = utm_point.northing + utm_point_offset.northing;
                utm_correct_point.zone = utm_zone;
                utm_correct_point.band = 'N';

                geographic_msgs::GeoPoint geo_point;
                geo_point = geodesy::toMsg(utm_correct_point);
                coordText += std::to_string(geo_point.longitude) + "," + std::to_string(geo_point.latitude) + "," + std::to_string(geo_point.altitude) + " ";
            }
            coordinates->SetText(coordText.c_str());
            point->InsertEndChild(coordinates);
        } else if (named_utm_point.type == NamedUTMPoint::LINE) {
            tinyxml2::XMLElement* lineString = doc.NewElement("LineString");
            placemark->InsertEndChild(lineString);
            tinyxml2::XMLElement* coordinates = doc.NewElement("coordinates");
            std::string coordText;
            for (auto& utm_point : named_utm_point.utm_points) {
                geodesy::UTMPoint utm_correct_point;
                utm_correct_point.altitude = utm_point.altitude + utm_point_offset.altitude;
                utm_correct_point.easting = utm_point.easting + utm_point_offset.easting;
                utm_correct_point.northing = utm_point.northing + utm_point_offset.northing;
                utm_correct_point.zone = utm_zone;
                utm_correct_point.band = 'N';

                geographic_msgs::GeoPoint geo_point;
                geo_point = geodesy::toMsg(utm_correct_point);
                coordText += std::to_string(geo_point.longitude) + "," + std::to_string(geo_point.latitude) + "," + std::to_string(geo_point.altitude) + " ";
            }
            coordinates->SetText(coordText.c_str());
            lineString->InsertEndChild(coordinates);
        } else if (named_utm_point.type == NamedUTMPoint::POLYGON) {
            tinyxml2::XMLElement* polygon = doc.NewElement("Polygon");
            placemark->InsertEndChild(polygon);
            tinyxml2::XMLElement* outerBoundaryIs = doc.NewElement("outerBoundaryIs");
            polygon->InsertEndChild(outerBoundaryIs);
            tinyxml2::XMLElement    * linearRing = doc.NewElement("LinearRing");
            outerBoundaryIs->InsertEndChild(linearRing);
            tinyxml2::XMLElement* coordinates = doc.NewElement("coordinates");
            std::string coordText;
            for (auto& utm_point : named_utm_point.utm_points) {
                geodesy::UTMPoint utm_correct_point;
                utm_correct_point.altitude = utm_point.altitude + utm_point_offset.altitude;
                utm_correct_point.easting = utm_point.easting + utm_point_offset.easting;
                utm_correct_point.northing = utm_point.northing + utm_point_offset.northing;
                utm_correct_point.zone = utm_zone;
                utm_correct_point.band = 'N';

                geographic_msgs::GeoPoint geo_point;
                geo_point = geodesy::toMsg(utm_correct_point);
                coordText += std::to_string(geo_point.longitude) + "," + std::to_string(geo_point.latitude) + "," + std::to_string(geo_point.altitude) + " ";
            }
            coordinates->SetText(coordText.c_str());
            linearRing->InsertEndChild(coordinates);
        } else {
            NAVIT_ROS_ERROR_STREAM("Unknown type.");
        }
    }
    doc.SaveFile(file_path.c_str());
}

void utm2map(std::vector<NamedUTMPoint>& named_utm_points) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        NAVIT_ROS_INFO_STREAM("Waiting for transform");
        listener.waitForTransform("utm", "map", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("utm", "map", ros::Time(0), transform);
        NAVIT_ROS_INFO_STREAM("Find tf.");
    } catch (tf::TransformException ex) {
        NAVIT_ROS_ERROR_STREAM(ex.what());
        return;
    }
    
    for (auto& named_utm_point : named_utm_points) {
        for (auto& utm_point : named_utm_point.utm_points) {
            tf::Vector3 utm_vector(utm_point.easting, utm_point.northing, 0);
            tf::Vector3 map_vector = transform * utm_vector;
            utm_point.easting = map_vector.x();
            utm_point.northing = map_vector.y();
        }
    }
}

void map2utm(std::vector<NamedUTMPoint>& named_utm_points) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        NAVIT_ROS_INFO_STREAM("Waiting for transform");
        listener.waitForTransform("map", "utm", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("map", "utm", ros::Time(0), transform);
        NAVIT_ROS_INFO_STREAM("Find tf.");
    } catch (tf::TransformException ex) {
        NAVIT_ROS_ERROR_STREAM(ex.what());
        return;
    }
    
    for (auto& named_utm_point : named_utm_points) {
        for (auto& utm_point : named_utm_point.utm_points) {
            tf::Vector3 utm_vector(utm_point.easting, utm_point.northing, 0);
            tf::Vector3 map_vector = transform * utm_vector;
            utm_point.easting = map_vector.x();
            utm_point.northing = map_vector.y();
        }
    }
}
void convertUtmToRosPath(const std::vector<geodesy::UTMPoint>& utm_points, nav_msgs::Path& path) {
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    for (auto& utm_point : utm_points) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = utm_point.easting;
        pose.pose.position.y = utm_point.northing;
        pose.pose.position.z = 0;
        path.poses.push_back(pose);
    }
}
}
}