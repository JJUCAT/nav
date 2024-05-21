
#include <prm_planner/debug_viewer.h>

namespace navit_planner {
void DebugViewer::publishRandomPoints(const std::map<int, Point>& points) {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& pair : points) {
        int index = pair.first;
        const Point& point = pair.second;
        // for debug
        //std::cout << boost::geometry::get<0>(point) << " " << boost::geometry::get<1>(point) << std::endl;

        visualization_msgs::Marker marker = createPointMarker(point, id);
        marker_array.markers.push_back(marker);
        id++;
    }

    marker_pub_.publish(marker_array);
}

void DebugViewer::publishEdges(const std::map<int, Point>& points, const std::map<int, std::pair<Point, Point>>& edges) {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& pair : edges) {
        int index = pair.first;
        const Point& p1 = pair.second.first;
        const Point& p2 = pair.second.second;

        visualization_msgs::Marker marker = createLineMarker(p1, p2, id);
        marker_array.markers.push_back(marker);
        id++;
    }

    marker_edges_pub_.publish(marker_array);
}

void DebugViewer::vieweGraphEdges(const std::map<int, std::tuple<Point, Point, double>>& graph) {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& pair : graph) {
        int index = pair.first;
        const Point& p1 = std::get<0>(pair.second);
        const Point& p2 = std::get<1>(pair.second);

        visualization_msgs::Marker marker = createLineMarker(p1, p2, id);
        marker_array.markers.push_back(marker);
        id++;
    }

    marker_edges_pub_.publish(marker_array);


}

std::vector<visualization_msgs::MarkerArray> DebugViewer::getEdges(MapInfo map_info) {
    // for (auto& area: map_info.areas) {
    //     PRM prm(area);
    //     // sample 200 points for each area...
    //     prm.samplePoints(20);

    //     for (int i = 0; i < map_info.connect_edges.size(); ++i) {
    //         for (int j = 0; j < map_info.connect_edges[i].size(); ++j) {
    //             std::cout << "x is " << map_info.connect_edges[i][j].x() << std::endl;
    //             std::cout << "y is " << map_info.connect_edges[i][j].y() << std::endl;

    //             prm.insertSamplePoints(map_info.connect_edges[i][j]);
    //         }
    //     }

    //     //std::map<int, Point> points = prm.getSamples();
    //     std::map<int, std::tuple<Point, Point, double>> edges = prm.connectPoints(0);
    //     std::cout << "edges size: " << edges.size() << std::endl;
    //     visualization_msgs::MarkerArray marker_array;
    //     int id = 0;

    //     for (const auto& pair : edges) {
    //         int index = pair.first;
    //         const Point& p1 = std::get<0>(pair.second);
    //         const Point& p2 = std::get<1>(pair.second);

    //         visualization_msgs::Marker marker = createLineMarker(p1, p2, id);
    //         marker_array.markers.push_back(marker);
    //         id++;
    //     }
    //     marker_array_vector_.push_back(marker_array);
    // }
    // return marker_array_vector_;
}

void DebugViewer::vieweGraphIndex(const std::map<int, std::tuple<Point, Point, double>>& graph) {
    visualization_msgs::MarkerArray markerArray;
    for (const auto& edge : graph) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "edges";
        marker.id = edge.first;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        // Position the text marker at the midpoint of the edge
        const auto& [point1, point2, weight] = edge.second;
        marker.pose.position.x = (point1.x() + point2.x()) / 2;
        marker.pose.position.y = (point1.y() + point2.y()) / 2;
        marker.pose.position.z = 1; // adjust this as needed

        marker.text = std::to_string(edge.first);

        marker.scale.z = 0.5; // size of the text
        marker.color.a = 1.0; // Make sure the text is visible
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        markerArray.markers.push_back(marker);
    }
    edges_num_marker_pub_.publish(markerArray);
}

void DebugViewer::publishEdges(std::vector<visualization_msgs::MarkerArray>& marker_array_vector) {
    for (int i = 0; i < marker_array_vector.size(); i++) {
        if (i == 0) {
            marker_edges_pub_1_.publish(marker_array_vector[i]);
        } else {
            marker_edges_pub_2_.publish(marker_array_vector[i]);
        }
    }
}

visualization_msgs::Marker DebugViewer::createPointMarker(const Point& point, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    return marker;
}

visualization_msgs::Marker DebugViewer::createLineMarker(const Point& p1, const Point& p2, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point msg_p1, msg_p2;
    msg_p1.x = p1.x();
    msg_p1.y = p1.y();
    msg_p2.x = p2.x();
    msg_p2.y = p2.y();
    marker.points.push_back(msg_p1);
    marker.points.push_back(msg_p2);
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    return marker;
}

void DebugViewer::publishMapPolygon(const Polygon& bounds) {
    visualization_msgs::Marker bounds_marker;
    bounds_marker.header.frame_id = "map";
    bounds_marker.header.stamp = ros::Time::now();
    bounds_marker.id = 0;
    bounds_marker.type = visualization_msgs::Marker::LINE_STRIP;
    bounds_marker.action = visualization_msgs::Marker::ADD;
    bounds_marker.scale.x = 0.05;
    bounds_marker.color.a = 1.0;
    bounds_marker.color.r = 0.0;
    bounds_marker.color.g = 0.0;
    bounds_marker.color.b = 1.0;

    // Outer loop
    for (const auto& point : bounds.outer()) {
        geometry_msgs::Point msg_point;
        msg_point.x = point.x();
        msg_point.y = point.y();
        bounds_marker.points.push_back(msg_point);
    }

    // Close the loop
    const auto& first_point = bounds.outer().front();
    geometry_msgs::Point msg_first_point;
    msg_first_point.x = first_point.x();
    msg_first_point.y = first_point.y();
    bounds_marker.points.push_back(msg_first_point);

    bounds_pub_.publish(bounds_marker);

    // Inner loops (if any)
    for (const auto& inner_ring : bounds.inners()) {
        visualization_msgs::Marker inner_marker;
        inner_marker.header.frame_id = "map";
        inner_marker.header.stamp = ros::Time::now();
        inner_marker.id = 0;
        inner_marker.type = visualization_msgs::Marker::LINE_STRIP;
        inner_marker.action = visualization_msgs::Marker::ADD;
        inner_marker.scale.x = 0.05;
        inner_marker.color.a = 1.0;
        inner_marker.color.r = 0.0;
        inner_marker.color.g = 1.0;
        inner_marker.color.b = 0.0;

        for (const auto& point : inner_ring) {
            geometry_msgs::Point msg_point;
            msg_point.x = point.x();
            msg_point.y = point.y();
            inner_marker.points.push_back(msg_point);
        }
        // Close the loop
        const auto& first_point = inner_ring.front();
        geometry_msgs::Point msg_first_point;
        msg_first_point.x = first_point.x();
        msg_first_point.y = first_point.y();
        inner_marker.points.push_back(msg_first_point);

        inner_pub_.publish(inner_marker);
    }
}

geometry_msgs::PolygonStamped DebugViewer::convertRingToRosPolygon(const Ring& ring) {

    geometry_msgs::PolygonStamped ros_polygon;
    ros_polygon.header.frame_id = "map";
    for (const auto& point : ring) {
        geometry_msgs::Point32 ros_point;
        ros_point.x = point.x();
        ros_point.y = point.y();
        ros_polygon.polygon.points.push_back(ros_point);
    }

    return ros_polygon;
}

void DebugViewer::initializePublishers(std::vector<Polygon> polygons) {
    std::cout << "polygons is " << polygons.size() << std::endl;
    for (int i = 0; i < polygons.size(); ++i) {
        std::string drivable_topic = "/show/drivable_polygon_" + std::to_string(i + 1);
        drivable_polygon_pubs_.push_back(nh_.advertise<geometry_msgs::PolygonStamped>(drivable_topic, 1));
    }

    for (int i = 0; i < polygons.size(); ++i) {
        for (auto hole : polygons[i].inners()) {
            std::string drivable_topic = "/show/non_drivable_polygon_" + std::to_string(i + 1);
            non_drivable_polygon_pubs_.push_back(nh_.advertise<geometry_msgs::PolygonStamped>(drivable_topic, 1));
        }
    }
}

void DebugViewer::publishMapPolygons(const std::vector<Polygon>& polygons) {
    // Convert each Polygon to a geometry_msgs::Polygon and publish it
    for (size_t i = 0, j = 0; i < polygons.size(); ++i) {
        const auto& polygon = polygons[i];
        // Publish outer ring
        geometry_msgs::PolygonStamped ros_polygon_outer = convertRingToRosPolygon(polygon.outer());
        drivable_polygon_pubs_[i].publish(ros_polygon_outer);
        
        // Publish each inner ring
        for (auto inner: polygon.inners()) {
            geometry_msgs::PolygonStamped ros_polygon_inner = convertRingToRosPolygon(inner);
            non_drivable_polygon_pubs_[j].publish(ros_polygon_inner);
            j++;
        }
    }
}
std::pair<int, std::tuple<Point, Point, double>>
DebugViewer::findClosestEdge(const std::map<int, std::tuple<Point, Point, double>>& edges, const Point& point) {
    double min_distance = std::numeric_limits<double>::max();
    std::pair<int, std::tuple<Point, Point, double>> closest_edge;

    for (const auto& edge : edges) {
        const Point& p1 = std::get<0>(edge.second);
        const Point& p2 = std::get<1>(edge.second);

        boost::geometry::model::linestring<Point> line;
        line.push_back(p1);
        line.push_back(p2);

        double distance = boost::geometry::distance(point, line);

        if (distance < min_distance) {
            min_distance = distance;
            closest_edge = edge;
        }
    }

    return closest_edge;
}
void DebugViewer::pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // Convert the ROS point to a Boost point
    Point point(msg->point.x, msg->point.y);
    
    // Find the closest edge to the point
    auto closest_edge = findClosestEdge(graph_, point);
    
    // Publish the closest edge
    publishEdge(closest_edge);
}

void DebugViewer::publishEdge(const std::pair<int, std::tuple<Point, Point, double>>& edge) {
    // Create a marker for the edge
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.
    marker.header.frame_id = "map";  // assuming the frame is map
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  
    marker.ns = "closest_edge";
    marker.id = edge.first;  // Assuming the id is the first element of the pair

    // Set the marker type.
    marker.type = visualization_msgs::Marker::LINE_LIST;

    // Set the marker action. Options are ADD, DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker 
    marker.scale.x = 0.1; // This is the width of the line, adjust as needed.

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    // Set the pose of the marker.
    marker.pose.orientation.w = 1.0; // Orientation is not important for LINE_LIST.

    // Now we add the two points that form the line
    geometry_msgs::Point p1, p2;
    p1.x = std::get<0>(edge.second).x();
    p1.y = std::get<0>(edge.second).y();
    p1.z = 0; // Assuming you're working in 2D

    p2.x = std::get<1>(edge.second).x();
    p2.y = std::get<1>(edge.second).y();
    p2.z = 0; // Assuming you're working in 2D

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    // Publish the marker
    closest_pub_.publish(marker);
}

void DebugViewer::publishResultMarker(const navit_core::SearchResultRoute& out_result, 
                                      std::map<int, std::tuple<int, Point, int, Point, float>> edges_with_ids) {
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    for (const auto& edge_id : out_result.via_points_order) {
        if (edges_with_ids.count(edge_id) > 0) {
            const auto& [id1, p1, id2, p2, weight] = edges_with_ids[edge_id];

            visualization_msgs::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = ros::Time::now();
            line.ns = "lines";
            line.id = marker_id++;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;

            // Set scale
            line.scale.x = 0.1;  // Line width

            // Set color
            line.color.a = 1.0;
            line.color.r = 0.0;
            line.color.g = 0.0;
            line.color.b = 1.0;

            // Define the line points
            geometry_msgs::Point point;
            point.x = p1.x();
            point.y = p1.y();
            line.points.push_back(point);

            point.x = p2.x();
            point.y = p2.y();
            line.points.push_back(point);

            marker_array.markers.push_back(line);
        }
    }
    result_path_marker_pub_.publish(marker_array);
}
} //namespace navit_planner