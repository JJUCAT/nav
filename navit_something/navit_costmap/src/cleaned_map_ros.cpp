/*
@brief: created by czk
*/
#include <navit_costmap/cleaned_layer.h>

/*
@brief: A costmap layer that marks the areas the robot has cleaned， which is different from general map layer
        it use a geomerty polygons to mark the cleaned area
@TODOS:
     1. Add a ROS service to enable/disable the layer
     2. Add a parameter to set the resolution of the layer
     3. Add a Ros service to reset the layer
*/
namespace navit_costmap
{
    void CleanedLayer::onInitialize(ros::NodeHandle& nh) {
        // Initialize the ROS service for enabling/disabling

        // enable_swept_layer_service_ = private_nh.advertiseService("enable_disable", &CleanedLayer::enableDisableService, this);
        // clear_swept_layer_service_ = private_nh.advertiseService("clear_swept_layer", &CleanedLayer::clearService, this);
        // get_swept_layer_service_ = private_nh.advertiseService("get_swept_layer", &CleanedLayer::getSweptService, this);
        nh_ = nh;

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("bounding_marker", 10);
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);
        cleaned_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("cleaned_occupancy_grid", 1);
        edge_paths_pub_ = nh.advertise<nav_msgs::Path>("cleaned_paths", 1);
        nh_.param("update_frequency", update_frequency_, 1.0);

        XmlRpc::XmlRpcValue footprint_xmlrpc;
        if (nh_.getParam("footprint", footprint_xmlrpc)) {
            if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                footprint_.clear();
                footprint_polygon_.outer().clear();

                for (int i = 0; i < footprint_xmlrpc.size(); ++i) {
                    if (footprint_xmlrpc[i].getType() == XmlRpc::XmlRpcValue::TypeArray &&
                        footprint_xmlrpc[i].size() == 2) {

                        geometry_msgs::Point point;
                        point.x = static_cast<double>(footprint_xmlrpc[i][0]);
                        point.y = static_cast<double>(footprint_xmlrpc[i][1]);

                        footprint_.push_back(point);

                        Point bg_point(point.x, point.y);
                        footprint_polygon_.outer().push_back(bg_point);
                    }
                }

                if (!footprint_polygon_.outer().empty()) {
                    footprint_polygon_.outer().push_back(footprint_polygon_.outer().front());
                }

                boost::geometry::correct(footprint_polygon_);
            }
        } else {
            ROS_WARN("The footprint parameter must be specified as a list of lists on the parameter server, but it was set to \"footprint\".");
            // use default footprint
        }


        enabled_ = true;
    }

    Polygon CleanedLayer::convertToBoostPolygon(const geometry_msgs::Polygon& ros_polygon) {
        Polygon boost_polygon;
        for(const auto& point : ros_polygon.points) {
        boost::geometry::append(boost_polygon, Point(point.x, point.y));
        }
        // Close the polygon by adding the first point at the end
        if (!ros_polygon.points.empty()) {
        boost::geometry::append(boost_polygon, Point(ros_polygon.points[0].x, ros_polygon.points[0].y));
        }
        return boost_polygon;
    }

    void CleanedLayer::setPolygonMap(geometry_msgs::Polygon bounding_polygon, std::vector<geometry_msgs::Polygon> forbidden_zones_polygons) {

        bounding_polygon_ = convertToBoostPolygon(bounding_polygon);

        // Convert forbidden_zones_polygons to Boost Geometry Polygons
        forbidden_zones_polygons_.clear();
        for (const auto& forbidden_polygon : forbidden_zones_polygons) {
            forbidden_zones_polygons_.push_back(convertToBoostPolygon(forbidden_polygon));
        }

        std::vector<Polygon> result_polygons;
        result_polygons.push_back(bounding_polygon_);

        for (const auto& forbidden_polygon : forbidden_zones_polygons_) {
            std::vector<Polygon> temp_result;
            for (const auto& existing_polygon : result_polygons) {
                std::vector<Polygon> output;
                boost::geometry::difference(existing_polygon, forbidden_polygon, output);
                temp_result.insert(temp_result.end(), output.begin(), output.end());
            }
            result_polygons = temp_result;
        }

        for (const auto& polygon : result_polygons) {
            whole_area_ += boost::geometry::area(polygon);
        }
        ROS_INFO("We cleaned %.2f m^2 \n", whole_area_);

        std_msgs::ColorRGBA bounding_color, forbidden_color, cleaned_color;

        bounding_color.r = 1.0; bounding_color.g = 0.0; bounding_color.b = 0.0; bounding_color.a = 0.5;  // 红色
        forbidden_color.r = 0.0; forbidden_color.g = 1.0; forbidden_color.b = 0.0; forbidden_color.a = 0.5;  // 绿色
        cleaned_color.r = 0.0; cleaned_color.g = 0.0; cleaned_color.b = 1.0; cleaned_color.a = 0.5;  // 蓝色

        // pub bounding_polygon_
        publishFilledPolygon(bounding_polygon_, 0, "bounding", bounding_color);

        // pub forbidden_zones_polygons_
        int id = 1;
        for (const auto& polygon : forbidden_zones_polygons_) {
            publishFilledPolygon(polygon, id++, "forbidden", forbidden_color);
        }

        // pub cleaned_polygons_
        id = 1;
        for (const auto& polygon : cleaned_polygons_) {
            publishFilledPolygon(polygon, id++, "cleaned", cleaned_color);
        }

        // // 计算 bounding_polygon_ 和 forbidden_zones_polygons_ 的差集
        // std::vector<Polygon> difference_polygons;
        // difference_polygons.push_back(bounding_polygon_);

        // for (const auto& forbidden_polygon : forbidden_zones_polygons_) {
        //     std::vector<Polygon> temp_result;
        //     for (const auto& existing_polygon : difference_polygons) {
        //         std::vector<Polygon> output;
        //         boost::geometry::difference(existing_polygon, forbidden_polygon, output);
        //         temp_result.insert(temp_result.end(), output.begin(), output.end());
        //     }
        //     difference_polygons = temp_result;
        // }
        // pub occupied grid map
        nav_msgs::OccupancyGrid grid = generateOccupancyGrid(map_resolution_);
        cleaned_grid_.info.height = grid.info.height;
        cleaned_grid_.info.width = grid.info.width;
        cleaned_grid_.info.resolution = grid.info.resolution;
        cleaned_grid_.info.origin = grid.info.origin;
        cleaned_grid_.header.frame_id = "map";
        // cleaned_grid_ = grid;

        map_pub_.publish(grid);


    }
    bool CleanedLayer::isPointInPolygon(Point point, Polygon polygon) {
        return boost::geometry::within(point, polygon);
    }

    bool CleanedLayer::isPointInAnyForbiddenZone(Point point, std::vector<Polygon> forbidden_zones) {
        for (const auto& zone : forbidden_zones) {
            if (isPointInPolygon(point, zone)) {
                return true;
            }
        }
        return false;
    }
    void CleanedLayer::publishFilledPolygon(const Polygon& polygon, int id, const std::string& ns, const std_msgs::ColorRGBA& color) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;

        std_msgs::ColorRGBA clean_color;
        clean_color.r = 0.0; clean_color.g = 0.0; clean_color.b = 1.0; clean_color.a = 0.5;  // 蓝色

        marker.color = clean_color;

        marker.pose.orientation.w = 1.0;

        for (const auto& point : polygon.outer()) {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = 0;

            marker.points.push_back(p);
        }

        // Close the loop
        if (!polygon.outer().empty()) {
            marker.points.push_back(marker.points.front());
        }

        marker_pub_.publish(marker);
    }

    // void CleanedLayer::updateGridWithFootprint(std::vector<int>& grid_data, int width, int height, double resolution, const Polygon& robot_footprint) {
    //     // 1. Get the bounding box of the robot_footprint
    //     Box box;
    //     boost::geometry::envelope(robot_footprint, box);

    //     // 2. Determine the range of grid cells possibly covered by the bounding box
    //     int min_x = std::max((int)((box.min_corner().x()) / resolution), 0);
    //     int max_x = std::min((int)((box.max_corner().x()) / resolution), width - 1);
    //     int min_y = std::max((int)((box.min_corner().y()) / resolution), 0);
    //     int max_y = std::min((int)((box.max_corner().y()) / resolution), height - 1);

    //     // 3. Update the values of the grid cells covered by the bounding box
    //     for (int x = min_x; x <= max_x; ++x) {
    //         for (int y = min_y; y <= max_y; ++y) {
    //             Point point(x * resolution, y * resolution);
    //             if (boost::geometry::within(point, robot_footprint)) {
    //                 grid_data[y * width + x] = 50;  // Special cost for robot footprint
    //             }
    //         }
    //     }
    // }

    bool CleanedLayer::makePlan(const cv::Mat& map, std::vector<nav_msgs::Path>& edge_paths, std::vector<Polygon>& clean_polygons)
    {
        // Existing logic for the edge path planning
        cv::Mat inverted_map;
        // cv::bitwise_not(map, inverted_map);  // Invert the map

        cv::Mat eroded;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                    cv::Size(2 * dilation_size_ + 1, 2 * dilation_size_ + 1),
                    cv::Point(dilation_size_, dilation_size_));

        cv::dilate(map, eroded, element);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> edge_points;

        cv::findContours(eroded, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::Point2f> centers;
        for (const auto& contour : contours) {
            // if (contour.size() < countour_threshold_)
            //     continue;

            nav_msgs::Path path;
            path.header.frame_id = "map";
            geometry_msgs::PoseStamped prev_pose;

            Polygon poly;
            double origin_x = cleaned_grid_.info.origin.position.x;
            double origin_y = cleaned_grid_.info.origin.position.y;
            for (const auto& pt : contour) {
                Point point_in_map_coords(
                    pt.x * map_resolution_ + origin_x,
                    (pt.y) * map_resolution_ + origin_y
                );
                boost::geometry::append(poly, point_in_map_coords);
            }

            boost::geometry::correct(poly);
            clean_polygons.push_back(poly);

            std::cout << "map.rows is " << map.rows << std::endl;
            for (size_t i = 0; i < contour.size(); ++i) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.y = contour[i].x * map_resolution_ + origin_x;
                pose.pose.position.x = ( contour[i].y) * map_resolution_ + origin_y;
                pose.pose.position.z = 0.0f;

                pose.pose.orientation.w = 1;
                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                // if (i > 0) {
                //     double yaw = std::atan2(pose.pose.position.y - prev_pose.pose.position.y,
                //                             pose.pose.position.x - prev_pose.pose.position.x);
                //     tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
                //     tf::quaternionTFToMsg(q, pose.pose.orientation);
                // } else if (contour.size() > 1) {
                //     double yaw = std::atan2((inverted_map.rows - contour[i+1].y) * map_resolution_ - pose.pose.position.y,
                //                             contour[i+1].x * map_resolution_ - pose.pose.position.x);
                //     tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
                //     tf::quaternionTFToMsg(q, pose.pose.orientation);
                // }

                path.poses.push_back(pose);
                prev_pose = pose;
            }

            edge_paths.push_back(path);

            // cv::Moments m = cv::moments(contour);
            // if (m.m00 != 0) {
            //     cv::Point2f center(m.m10 / m.m00 * map_resolution_, (inverted_map.rows - m.m01 / m.m00) * map_resolution_);
            //     centers.push_back(center);
            //     geometry_msgs::Vector3 vec;
            //     vec.x = center.x;
            //     vec.y = center.y;
            //     vec.z = 0.0f;
            //     center_points.push_back(vec);
            // }
        }

        for (size_t i = 0; i < centers.size(); ++i) {
            for (size_t j = i + 1; j < centers.size(); ++j) {
                double distance = cv::norm(centers[i] - centers[j]);
                // std::cout << "Distance between center " << i << " and center " << j << " is: " << distance << std::endl;
            }
        }
        return true;
    }

    void CleanedLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                    Polygon& robot_polygon) {
        robot_polygon.clear();
        Point robot_point(robot_x, robot_y);
        if (!boost::geometry::within(robot_point, bounding_polygon_)) {
            return;
        }

        std::vector<Point> rectangle_points;
        double length = 1.0, width = 1.0;

        for (int i = 0; i < 4; ++i) {
            double sign_x = (i == 0 || i == 3) ? 1 : -1;
            double sign_y = (i < 2) ? 1 : -1;

            double corner_x = robot_x + sign_x * length / 2 * cos(robot_yaw) - sign_y * width / 2 * sin(robot_yaw);
            double corner_y = robot_y + sign_x * length / 2 * sin(robot_yaw) + sign_y * width / 2 * cos(robot_yaw);

            rectangle_points.push_back(Point(corner_x, corner_y));
        }

        robot_polygon.outer().push_back(rectangle_points[0]);
        robot_polygon.outer().push_back(rectangle_points[3]);
        robot_polygon.outer().push_back(rectangle_points[2]);
        robot_polygon.outer().push_back(rectangle_points[1]);
        robot_polygon.outer().push_back(rectangle_points[0]);

        // convert robot_polygon to grid map coordinate
        Polygon grid_robot_polygon;
        for (auto& point : robot_polygon.outer()) {
            Point grid_point((point.x() - bounding_polygon_.outer()[0].x())/cleaned_grid_.info.resolution,
                             (point.y() - bounding_polygon_.outer()[0].y())/cleaned_grid_.info.resolution);
            grid_robot_polygon.outer().push_back(grid_point);
        }

        boost::geometry::model::box<Point> bounding_box;
        boost::geometry::envelope(grid_robot_polygon, bounding_box);

        int min_x = static_cast<int>(std::floor(bounding_box.min_corner().x()));
        int min_y = static_cast<int>(std::floor(bounding_box.min_corner().y()));
        int max_x = static_cast<int>(std::ceil(bounding_box.max_corner().x()));
        int max_y = static_cast<int>(std::ceil(bounding_box.max_corner().y()));

        int grid_width = cleaned_grid_.info.width;
        int grid_height = cleaned_grid_.info.height;

        cleaned_grid_.data.resize(grid_height * grid_width);

        min_x = std::max(0, min_x);
        min_y = std::max(0, min_y);
        max_x = std::min(grid_width - 1, max_x);
        max_y = std::min(grid_height - 1, max_y);

        for (int x = std::max(0, min_x); x <= std::min(grid_width - 1, max_x); ++x) {
            for (int y = std::max(0, min_y); y <= std::min(grid_height - 1, max_y); ++y) {
                Point grid_point(x, y);

                if (boost::geometry::within(grid_point, grid_robot_polygon)) {
                    int index = y * grid_width + x;
                    cleaned_grid_.data[index] = 123;
                }
            }
        }

        cleaned_map_pub_.publish(cleaned_grid_);

        cv::Mat map = cv::Mat(grid_height, grid_width, CV_8UC1);

        for(int i = 0; i < grid_height -1 ; ++i) {
            for(int j = 0; j < grid_width -1 ; ++j) {
                int index = i * grid_width + j;
                map.at<uchar>(i, j) = cleaned_grid_.data[index];
            }
        }
        std::vector<nav_msgs::Path> clean_boxes;
        std::vector<Polygon> clean_polygons;

        makePlan(map, clean_boxes, clean_polygons);

        // pub clean_boxes

        for (const auto& path : clean_boxes) {
            nav_msgs::Path ros_path = path;
            ros_path.header.stamp = ros::Time::now();
            ros_path.header.frame_id = "map";

            edge_paths_pub_.publish(ros_path);
        }
        // pub clean_polygons
        int id = 5;
        std_msgs::ColorRGBA clean_color;
        clean_color.r = 0.0; clean_color.g = 0.0; clean_color.b = 1.0; clean_color.a = 0.5;  // 蓝色

        for (const auto& polygon : clean_polygons) {
            publishFilledPolygon(polygon, id++, "cleaned", clean_color);
        }
        double clean_area = calCleanedArea(clean_polygons);
    }
    //
    nav_msgs::OccupancyGrid CleanedLayer::generateOccupancyGrid(double resolution) {
        nav_msgs::OccupancyGrid grid;
        grid.info.resolution = resolution;

        grid.info.origin.position.x = bounding_polygon_.outer()[0].x();
        grid.info.origin.position.y = bounding_polygon_.outer()[0].y();
        grid.info.origin.orientation.w = 1.0;

        // Calculate bounding box for bounding_polygon_
        boost::geometry::model::box<Point> box;
        boost::geometry::envelope(bounding_polygon_, box);
        grid.info.width = std::ceil((box.max_corner().x() - box.min_corner().x()) / resolution);
        grid.info.height = std::ceil((box.max_corner().y() - box.min_corner().y()) / resolution);

        grid.data.resize(grid.info.width * grid.info.height, -1);

        // Iterate through each grid cell
        for (int x = 0; x < grid.info.width; ++x) {
            for (int y = 0; y < grid.info.height; ++y) {
                Point point(box.min_corner().x() + x * grid.info.resolution, box.min_corner().y() + y * grid.info.resolution);
                if (isPointInPolygon(point, bounding_polygon_) && !isPointInAnyForbiddenZone(point, forbidden_zones_polygons_)) {
                    grid.data[y * grid.info.width + x] = 0;  // Free
                } else {
                    grid.data[y * grid.info.width + x] = 100;  // Occupied
                }


            }
        }
        return grid;
    }
    //cleaned map cycle
    void CleanedLayer::mapUpdateLoop(double frequency)
    {
        ros::NodeHandle nh;
        ros::Rate r(frequency);
        double clean_area = 0.0;
        tf::StampedTransform tf_transform_;
        while (nh.ok()) {
            try {
                tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(0.5));
                tf_listener_.lookupTransform("map", "base_link", ros::Time(0), tf_transform_);
            } catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
                continue;
            }
            ros::Time last_time = ros::Time::now();
            updateBounds(tf_transform_.getOrigin().x(), tf_transform_.getOrigin().y(), tf::getYaw(tf_transform_.getRotation()), footprint_polygon_);


            r.sleep();
        }
    }

    double CleanedLayer::calCleanedArea(const std::vector<Polygon>& cleaned_areas) {
        double total_area = 0.0;
        for (const auto& polygon : cleaned_areas) {
            double area = boost::geometry::area(polygon);
            total_area += std::abs(area);
        }
        return total_area;
    }

    bool CleanedLayer::haveIntersection(const Polygon& poly1, const Polygon& poly2) {
        return boost::geometry::intersects(poly1, poly2);
    }



    Polygon CleanedLayer::mergePolygons(const Polygon& poly1, const Polygon& poly2) {
        std::vector<Polygon> output;
        boost::geometry::union_(poly1, poly2, output);

        if (!output.empty()) {
            return output[0];
        }

        return Polygon();
    }
    // nav_msgs::OccupancyGrid CleanedLayer::getCharMap() {
    //     nav_msgs::OccupancyGrid grid;
    //     grid.header.frame_id = layered_costmap_->getGlobalFrameID();
    //     grid.header.stamp = ros::Time::now();
    //     grid.info.resolution = resolution_;
    //     grid.info.width = getSizeInCellsX();
    //     grid.info.height = getSizeInCellsY();
    //     grid.info.origin.position.x = getOriginX();
    //     grid.info.origin.position.y = getOriginY();
    //     grid.info.origin.orientation.w = 1.0;
    //     grid.data.resize(grid.info.width * grid.info.height);
    //     for(unsigned int i = 0; i < grid.data.size(); ++i) {
    //     grid.data[i] = costmap_[i];
    //     }
    //     return grid;
    // }
} // navit costmap