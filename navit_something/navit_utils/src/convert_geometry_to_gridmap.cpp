#include <ros/ros.h>
#include <navit_utils/GetSelection.h>
#include <navit_utils/GetPolylines.h>
#include <navit_utils/GetPoints.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_polygon_node");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    ros::ServiceClient clientSelection = nh.serviceClient<navit_utils::GetSelection>("get_selection");
    ros::ServiceClient clientConnectionLine = nh.serviceClient<navit_utils::GetPolylines>("get_connection_line");
    ros::ServiceClient clientWorkPoints = nh.serviceClient<navit_utils::GetPoints>("get_work_points");

    navit_utils::GetSelection srvSelection;
    navit_utils::GetPolylines srvConnectionLine;
    navit_utils::GetPoints srvWorkPoints;
    std::vector<std::vector<cv::Point2f>> polygons;

    if (clientSelection.call(srvSelection) && clientConnectionLine.call(srvConnectionLine) && clientWorkPoints.call(srvWorkPoints)) {
        // Assume the first polygon is the outer boundary and the rest are inner boundaries
        for (const auto& poly : srvSelection.response.selection[0].polygons) {
            std::vector<cv::Point2f> polygon_points;
            for (const auto& point : poly.polygon.points) {
                polygon_points.push_back(cv::Point2f(point.x, point.y));
            }
            polygons.push_back(polygon_points);
        }

        // Calculate the bounding box for all polygons
        std::vector<cv::Point2f> all_points;
        for (const auto& polygon : polygons) {
            all_points.insert(all_points.end(), polygon.begin(), polygon.end());
        }
        cv::Rect2f bounding_box = cv::boundingRect(all_points);

        float resolution = 0.05; 
        cv::Size image_size(std::ceil(bounding_box.width / resolution), std::ceil(bounding_box.height / resolution));
        cv::Mat image = cv::Mat::zeros(image_size, CV_8UC1);
        for (size_t i = 0; i < polygons.size(); ++i) {
            std::vector<cv::Point> pixel_points;
            for (const auto& point : polygons[i]) {
                pixel_points.push_back(cv::Point(static_cast<int>((point.x - bounding_box.x) / resolution + 0.5), 
                                                 static_cast<int>((point.y - bounding_box.y) / resolution + 0.5)));
                std::cout<< "point" << static_cast<int>((point.x - bounding_box.x) / resolution + 0.5) << " " << static_cast<int>((point.y - bounding_box.y) / resolution + 0.5) << std::endl;
            }

            if (!pixel_points.empty()) {
                std::vector<std::vector<cv::Point>> polys(1, pixel_points); // create a vector of polygon
                cv::fillPoly(image, polys, cv::Scalar(i == 0 ? 254 : 0)); // fill the polygons, 255 for outer boundary, 0 for inner boundaries
            } else {
                std::cout << "Warning: pixel_points is empty for polygon " << i << std::endl;
            }
        }

        // Create a nav_msgs/OccupancyGrid message
        nav_msgs::OccupancyGrid grid;
        grid.info.resolution = resolution;
        grid.info.width = image_size.width;
        grid.info.height = image_size.height;
        grid.info.origin.position.x = bounding_box.x;
        grid.info.origin.position.y = bounding_box.y;
        grid.info.origin.orientation.w = 1.0; // Identity quaternion
        grid.data.resize(grid.info.width * grid.info.height);
        for (int i = 0; i < image_size.height; ++i) {
            for (int j = 0; j < image_size.width; ++j) {
                grid.data[i * grid.info.width + j] = image.at<uint8_t>(i, j) > 0 ? 0 : 100;
            }
        }

        // Create publisher for the OccupancyGrid
        
        sleep(2);
        // Publish the grid
        map_pub.publish(grid);
        ros::spin();
    } else {
        ROS_ERROR("Failed to call service get_selection or get_connection_line or get_work_points");
        return 1;
    }

    return 0;
}