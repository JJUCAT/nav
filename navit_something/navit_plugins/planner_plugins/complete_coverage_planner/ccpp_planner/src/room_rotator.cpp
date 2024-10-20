#include "room_rotator.h"

#include <map>

void RoomRotator::rotateRoom(const cv::Mat& room_map, cv::Mat& rotated_room_map, const cv::Mat& R, const cv::Rect& bounding_rect)
{
	// rotate the image
	cv::warpAffine(room_map, rotated_room_map, R, bounding_rect.size(), cv::INTER_AREA);
	cv::threshold(rotated_room_map, rotated_room_map, 127, 255, cv::THRESH_BINARY);
}

// compute the affine rotation matrix for rotating a room into parallel alignment with x-axis (longer side of the room is aligned with x-axis)
double RoomRotator::computeRoomRotationMatrix(const cv::Mat& room_map, cv::Mat& R, cv::Rect& bounding_rect,
		const double map_resolution, const cv::Point* center, const double rotation_offset)
{
	// rotation angle of the map s.t. the most occurring gradient is in 90 degree to the x-axis
	double rotation_angle = computeRoomMainDirection(room_map, map_resolution) + rotation_offset;
	std::cout << "RoomRotator::computeRoomRotationMatrix: main rotation angle: " << rotation_angle << std::endl;

	cv::Point center_of_rotation;
	if (center == 0)
	{
		std::vector < std::vector<cv::Point> > contour;
		cv::Mat contour_map = room_map.clone();
		cv::findContours(contour_map, contour, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
		// get the moment--> for a given map, there should only be one contour
		cv::Moments moment = cv::moments(contour[0], false);
		// calculate rotation center
		center_of_rotation = cv::Point(moment.m10/moment.m00 , moment.m01/moment.m00 );
	}
	else
		center_of_rotation = *center;

	// compute rotation
	R = cv::getRotationMatrix2D(center_of_rotation, (rotation_angle*180)/CV_PI, 1.0);

	// determine bounding rectangle to find the size of the new image
	bounding_rect = cv::RotatedRect(center_of_rotation, room_map.size(), (rotation_angle*180)/CV_PI).boundingRect();

	// adjust transformation matrix
	R.at<double>(0,2) += bounding_rect.width/2.0 - center_of_rotation.x;
	R.at<double>(1,2) += bounding_rect.height/2.0 - center_of_rotation.y;

	return rotation_angle;
}

double RoomRotator::computeRoomMainDirection(const cv::Mat& room_map, const double map_resolution)
{
	// compute Hough transform on edge image of the map
	cv::Mat edge_map;
	cv::Canny(room_map, edge_map, 50, 150, 3);


	std::vector<cv::Vec4i> lines;
	double min_line_length = 1.0;	// in [m]
	for (; min_line_length > 0.1; min_line_length -= 0.2)
	{
		cv::HoughLinesP(edge_map, lines, 1, CV_PI/180, min_line_length/map_resolution, min_line_length/map_resolution, 1.5*min_line_length/map_resolution);
		cv::Mat room_hough = edge_map.clone();
		for (size_t i=0; i<lines.size(); ++i)
		{
			cv::Point p1(lines[i][0], lines[i][1]), p2(lines[i][2], lines[i][3]);
			cv::line(room_hough, p1, p2, cv::Scalar(128), 2);
		}
		if (lines.size() >= 4)
			break;
	}
	// setup a histogram on the line directions weighted by their length to determine the major direction
	Histogram<double> direction_histogram(0, CV_PI, 36);
	for (size_t i=0; i<lines.size(); ++i)
	{
		double dx = lines[i][2] - lines[i][0];
		double dy = lines[i][3] - lines[i][1];
		if(dy*dy+dx*dx > 0.0)
		{
			double current_direction = std::atan2(dy, dx);
			while (current_direction < 0.)
				current_direction += CV_PI;
			while (current_direction > CV_PI)
				current_direction -= CV_PI;
			direction_histogram.addData(current_direction, sqrt(dy*dy+dx*dx));
		}
	}
	return direction_histogram.getMaxBinPreciseVal();
}

void RoomRotator::transformPathBackToOriginalRotation(const std::vector<cv::Point>& fov_middlepoint_path, std::vector<geometry_msgs::Pose2D>& path_fov_poses, const cv::Mat& R)
{
	path_fov_poses.clear();

	// transform the calculated path back to the originally rotated map
	cv::Mat R_inv;
	cv::invertAffineTransform(R, R_inv);
	std::vector<cv::Point> fov_middlepoint_path_transformed;
	cv::transform(fov_middlepoint_path, fov_middlepoint_path_transformed, R_inv);

	// create poses with an angle
	transformPointPathToPosePath(fov_middlepoint_path_transformed, path_fov_poses);
}

void RoomRotator::transformPointPathToPosePath(const std::vector<cv::Point>& point_path, std::vector<geometry_msgs::Pose2D>& pose_path)
{
	// create poses with an angle
	for(size_t point_index = 0; point_index < point_path.size(); ++point_index)
	{
		// get the vector from the previous to the current point
		const cv::Point& current_point = point_path[point_index];

		// add the next navigation goal to the path
		geometry_msgs::Pose2D current_pose;
		current_pose.x = current_point.x;
		current_pose.y = current_point.y;
		current_pose.theta = 0.;
		cv::Point vector(0,0);
		if (point_index > 0)
		{
			// compute the direction as the line from the previous point to the current point
			cv::Point previous_point = point_path[point_index-1];
			vector = current_point - previous_point;
		}
		else if (point_path.size() >= 2)
		{
			// for the first point take the direction between first and second point
			cv::Point next_point = point_path[point_index+1];
			vector = next_point - current_point;
		}
		// only sample different points
		if (vector.x!=0 || vector.y!=0)
		{
			current_pose.theta = std::atan2(vector.y, vector.x);
			pose_path.push_back(current_pose);
		}
	}
}

void RoomRotator::transformPointPathToPosePath(std::vector<geometry_msgs::Pose2D>& pose_path)
{
	// create point vector
	std::vector<cv::Point> point_path;
	for (size_t i=0; i<pose_path.size(); ++i)
		point_path.push_back(cv::Point(pose_path[i].x, pose_path[i].y));

	// create poses with an angle
	pose_path.clear();
	transformPointPathToPosePath(point_path, pose_path);
}

int RoomRotator::removeUnconnectedRoomParts(cv::Mat& room_map)
{
    // remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
    // create new map with segments labeled by increasing labels from 1,2,3,...
    int free_num = 0;
    cv::Mat room_map_int(room_map.rows, room_map.cols, CV_8UC1);
    for (int v=0; v<room_map.rows; ++v)
    {
        for (int u=0; u<room_map.cols; ++u)
        {
            if (room_map.at<uchar>(v,u) == 255)
            {
                room_map_int.at<uchar>(v,u) = 255;
                free_num++;
            }
            else
                room_map_int.at<uchar>(v,u) = 0;
        }
    }

    if(free_num <10)
    {
        return 1;
    }

    std::map<int, int> area_to_label_map;	// maps area=number of segment pixels (keys) to the respective label (value)
    int label = 100;
    for (int v=0; v<room_map_int.rows; ++v)
    {
        for (int u=0; u<room_map_int.cols; ++u)
        {
            if (room_map_int.at<uchar>(v,u) == 255)
            {
                const int area = cv::floodFill(room_map_int, cv::Point(u,v), cv::Scalar(label), 0, 0, 0, 8 | cv::FLOODFILL_FIXED_RANGE);
                area_to_label_map[area] = label;
                ++label;
                const int l=area_to_label_map.rbegin()->first;
            }
        }
    }

    // remove all room pixels from room_map which are not accessible
    const int label_of_biggest_room = area_to_label_map.rbegin()->second;
    for (int v=0; v<room_map.rows; ++v)
        for (int u=0; u<room_map.cols; ++u)
            if (room_map_int.at<uchar>(v,u) != label_of_biggest_room)
                room_map.at<uchar>(v,u) = 0;
    return 0;
}
