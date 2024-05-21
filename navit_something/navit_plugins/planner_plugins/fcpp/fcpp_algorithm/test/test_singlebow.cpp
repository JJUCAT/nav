/*
 * test_main.cpp
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#include "fcpp_algorithm/CoveragePlanner.h"
#include "fcpp_algorithm/common/def.h"

#define USE_SINGLE_COMPUTE

class MouseParams {
public:
	MouseParams(const cv::Mat &img_) {
		img = img_.clone();
		point.x_ = -1;
		point.y_ = -1;
		point.is_transfer_point = false;
	}
	polygon_coverage_planning::Point point;
	cv::Mat img;
};

void onMouseHandle(int event, int x, int y, int flags, void *params) {
	MouseParams *mp = (MouseParams*) params;
	switch (event) {
	case cv::EVENT_LBUTTONDOWN: {
		cv::circle(mp->img, cv::Point(x, y), 3, cv::Scalar(0, 64, 255), -1);
		cv::imshow("select start point", mp->img);
		std::cout << "<start point> x: " << x << ", y: " << y << std::endl;
		mp->point.x_ = x;
		mp->point.y_ = y;
		mp->point.is_transfer_point = false;
		break;
	}
	default: {
		break;
	}
	}
}

polygon_coverage_planning::Point getStartingPoint(cv::Mat &img) {
	MouseParams params(img);
	params.img = img.clone();
	cv::namedWindow("select start point", cv::WINDOW_AUTOSIZE);
	cv::imshow("select start point", params.img);
	cv::setMouseCallback("select start point", onMouseHandle,
			(void*) &(params));
	cv::waitKey();
	polygon_coverage_planning::Point point = params.point;
	img = params.img.clone();
	cv::destroyWindow("select start point");
	return point;
}

int main(int argv, char **argc) {
	polygon_coverage_planning::CoveragePlanner test;
	polygon_coverage_planning::CoveragePlannerConfig config;
	config.decompose_type = 0;
	config.sweep_step = 5;
	config.counter_clockwise = true;
	test.init(config);
	LOG(INFO) << "init success";
	if (argv != 2) {
		LOG(INFO) << "the input map is not exist";
		return -1;
	}
	std::string map(argc[1]);
	cv::Mat img = cv::imread(map.c_str());
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	cv::Mat img_ = gray.clone();
	cv::threshold(img_, img_, 250, 255, 0);
	cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
			cv::Size(10, 10), cv::Point(-1, -1)); // size: robot radius
	cv::morphologyEx(img_, img_, cv::MORPH_ERODE, erode_kernel);
	cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
			cv::Size(5, 5), cv::Point(-1, -1));
	cv::morphologyEx(img_, img_, cv::MORPH_OPEN, open_kernel);
	std::vector<std::vector<cv::Point>> cnts;
	std::vector<cv::Vec4i> hierarchy; // index: next, prev, first_child, parent
	cv::findContours(img_, cnts, hierarchy, cv::RETR_TREE,
			cv::CHAIN_APPROX_SIMPLE);
	std::vector<int> cnt_indices(cnts.size());
	std::iota(cnt_indices.begin(), cnt_indices.end(), 0);
	std::sort(cnt_indices.begin(), cnt_indices.end(),
			[&cnts](int lhs, int rhs) {
				return cv::contourArea(cnts[lhs]) > cv::contourArea(cnts[rhs]);
			});
	int ext_cnt_idx = cnt_indices.front();
	std::vector<std::vector<cv::Point>> contours;
	contours.emplace_back(cnts[ext_cnt_idx]);
	for (unsigned int i = 0; i < hierarchy.size(); i++) {
		if (hierarchy[i][3] == ext_cnt_idx) {
			contours.emplace_back(cnts[i]);
		}
	}
	std::vector<std::vector<cv::Point>> polys;
	std::vector<cv::Point> poly;
	for (auto &contour : contours) {
		cv::approxPolyDP(contour, poly, 3, true);
		polys.emplace_back(poly);
		poly.clear();
	}
	std::vector<cv::Point> outer_poly = polys.front();
	polys.erase(polys.begin());
	std::vector<std::vector<cv::Point>> inner_polys = polys;
	std::vector<polygon_coverage_planning::Point> outer_polygon;
	std::vector<std::vector<polygon_coverage_planning::Point>> innner_polygon(
			inner_polys.size());
	for (const auto &point : outer_poly) {
		outer_polygon.push_back(
				polygon_coverage_planning::Point(point.x, point.y));
	}
	for (unsigned int i = 0; i < inner_polys.size(); i++) {
		for (const auto &point : inner_polys[i]) {
			innner_polygon[i].push_back(
					polygon_coverage_planning::Point(point.x, point.y));
		}
	}
	polygon_coverage_planning::Point start = getStartingPoint(img);
	LOG(INFO) << "get start pose success";
	test.Update(outer_polygon, innner_polygon);
	LOG(INFO) << "update success";
	std::vector<polygon_coverage_planning::Point> way_points;
	test.SolveSingleBow(start);
	test.getWayPoint(way_points);
	LOG(INFO) << "solve success";
	// plot for test
	cv::Point p1, p2;
	cv::Mat temp_img = img.clone();
	cv::namedWindow("cover", cv::WINDOW_NORMAL);
	cv::imshow("cover", temp_img);
	for (size_t i = 1; i < way_points.size(); ++i) {
		if (way_points[i].is_transfer_point
				|| way_points[i - 1].is_transfer_point) {
			continue;
		}
		p1 = cv::Point(way_points[i].x_, way_points[i].y_);
		p2 = cv::Point(way_points[i - 1].x_, way_points[i - 1].y_);
		cv::line(temp_img, p1, p2, cv::Scalar(0, 64, 255));
		cv::namedWindow("cover", cv::WINDOW_NORMAL);
		cv::imshow("cover", temp_img);
		cv::waitKey(50);
	}
	LOG(INFO) << "plot success";
	// compute the coverage rate and repeated rate
	// compute the unprocessed contour
	cv::Mat img_for_counter = gray.clone();
	cv::threshold(img_for_counter, img_for_counter, 250, 255, 0);
	std::vector<std::vector<cv::Point>> cnts_for_compute;
	std::vector<cv::Vec4i> hierarchy_compute; // index: next, prev, first_child, parent
	cv::findContours(img_for_counter, cnts_for_compute, hierarchy_compute,
			cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	std::vector<int> cnt_indices_compute(cnts_for_compute.size());
	std::iota(cnt_indices_compute.begin(), cnt_indices_compute.end(), 0);
	std::sort(cnt_indices_compute.begin(), cnt_indices_compute.end(),
			[&cnts_for_compute](int lhs, int rhs) {
				return cv::contourArea(cnts_for_compute[lhs])
						> cv::contourArea(cnts_for_compute[rhs]);
			});
	ext_cnt_idx = cnt_indices_compute.front();
	std::vector<std::vector<cv::Point>> contours_for_compute;
	contours_for_compute.emplace_back(cnts_for_compute[ext_cnt_idx]);
	for (unsigned int i = 0; i < hierarchy_compute.size(); i++) {
		if (hierarchy_compute[i][3] == ext_cnt_idx) {
			contours_for_compute.emplace_back(cnts_for_compute[i]);
		}
	}
	std::vector<std::vector<cv::Point>> polys_for_compute;
	std::vector<cv::Point> poly_for_compute;
	for (auto &contour : contours_for_compute) {
		cv::approxPolyDP(contour, poly_for_compute, 3, true);
		polys_for_compute.emplace_back(poly_for_compute);
		poly_for_compute.clear();
	}
	std::vector<cv::Point> outer_poly_compute = polys_for_compute.front();
	polys_for_compute.erase(polys_for_compute.begin());
	std::vector<std::vector<cv::Point>> inner_polys_compute = polys_for_compute;

	// 1）compute the boundary area
	double boundary_area = cv::contourArea(outer_poly_compute);
	// 2) compute the holes area
	double hole_area = 0.0;
	for (auto &hole : inner_polys_compute) {
		hole_area += cv::contourArea(hole);
	}
	// 3) compute the coverage area and repeated area
	double coveratge_area = 0.0;
	double repeated_area = 0.0;
	cv::Mat img_for_compute = img.clone();
	cv::Mat data = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	for (size_t i = 1; i < way_points.size(); ++i) {
		if (way_points[i].is_transfer_point
				|| way_points[i - 1].is_transfer_point) {
			continue;
		}
		p2 = cv::Point(way_points[i].x_, way_points[i].y_);
		p1 = cv::Point(way_points[i - 1].x_, way_points[i - 1].y_);
		// set the blackboard is blank
		cv::Mat img_for_compute_mirror = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
		for (int row = 0; row != img_for_compute_mirror.rows; ++row)
			for (int col = 0; col != img_for_compute_mirror.cols; ++col)
				img_for_compute_mirror.at<uchar>(row, col) = 255;

		cv::line(img_for_compute_mirror, p1, p2, cv::Scalar(0, 64, 255), 5);
		std::vector<std::vector<cv::Point>> contours_mirror;
		std::vector<cv::Vec4i> hierarchy_mirror;
		cv::findContours(img_for_compute_mirror, contours_mirror, hierarchy_mirror, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
		assert(contours_mirror.size() == 2);
		for (int row = 0; row != img_for_compute.rows; ++row)
			for (int col = 0; col != img_for_compute.cols; ++col) {
				int distance = cv::pointPolygonTest(contours_mirror[1], cv::Point2f(row, col), false);
				if (distance == 1) {
					if(img_for_compute.at<cv::Vec3b>(row, col)[0] == 0
						&& img_for_compute.at<cv::Vec3b>(row, col)[1] == 64
						&& img_for_compute.at<cv::Vec3b>(row, col)[2] == 255){
						data.at<uchar>(row,col) = 1; //repeated cell
					}
				}
			}
		cv::line(img_for_compute, p1, p2, cv::Scalar(0, 64, 255), 5);
	}
	for (int i = 0; i != img_for_compute.cols; ++i) {
		for (int j = 0; j != img_for_compute.rows; ++j) {
			if (img_for_compute.at<cv::Vec3b>(i, j)[0] == 0
					&& img_for_compute.at<cv::Vec3b>(i, j)[1] == 64
					&& img_for_compute.at<cv::Vec3b>(i, j)[2] == 255) {
				coveratge_area += 1.0;
				if(data.at<uchar>(i,j) == 1){
					repeated_area +=1.0;
				}
			}
		}
	}
	LOG(INFO) << "boundary area is: " << boundary_area
			<< ", obstacle area is: " << hole_area << ", coverage area is:"
			<< coveratge_area << ", repeated area is:" << repeated_area;
	LOG(INFO) << "coverage rate is: "
			<< coveratge_area / (boundary_area - hole_area);
	LOG(INFO) << "repeated rate is: "
			<< repeated_area / (boundary_area - hole_area);
	pause();
	return 0;
}
