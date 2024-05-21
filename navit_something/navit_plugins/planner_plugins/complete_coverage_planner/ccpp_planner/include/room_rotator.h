#pragma once

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "histogram.h"

 namespace geometry_msgs {

 	struct Pose2D {
 		double x = 0;
 		double y = 0;
 		double theta = 0;
 	};

 }

class RoomRotator
{
public:
	RoomRotator() {}

	void rotateRoom(const cv::Mat& room_map, cv::Mat& rotated_room_map, const cv::Mat& R, const cv::Rect& bounding_rect);

	double computeRoomRotationMatrix(const cv::Mat& room_map, cv::Mat& R, cv::Rect& bounding_rect, const double map_resolution,
			const cv::Point* center=0, const double rotation_offset=0.);

	// computes the major direction of the walls from a map (preferably one room)
	double computeRoomMainDirection(const cv::Mat& room_map, const double map_resolution);

	// transforms a vector of points back to the original map and generates poses
	void transformPathBackToOriginalRotation(const std::vector<cv::Point>& fov_middlepoint_path, std::vector<geometry_msgs::Pose2D>& path_fov_poses, const cv::Mat& R);

	// converts a point path to a pose path with angles
	void transformPointPathToPosePath(const std::vector<cv::Point>& point_path, std::vector<geometry_msgs::Pose2D>& pose_path);

	// converts a point path to a pose path with angles, the points are already stored in pose_path
	void transformPointPathToPosePath(std::vector<geometry_msgs::Pose2D>& pose_path);
	
    int removeUnconnectedRoomParts(cv::Mat& room_map);
};
