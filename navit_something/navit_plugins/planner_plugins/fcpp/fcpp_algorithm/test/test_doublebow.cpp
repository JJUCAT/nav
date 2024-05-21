/*
 * test_main.cpp
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#include "fcpp_algorithm/CoveragePlanner.h"
#include "fcpp_algorithm/common/def.h"

class MouseParams {
public:
	MouseParams(const cv::Mat &img_) {
		img = img_.clone();
		point.x_ = -1;
		point.y_ = -1;
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

int main(int argv ,char **argc) {

	polygon_coverage_planning::CoveragePlanner test;
	polygon_coverage_planning::CoveragePlannerConfig config;
	config.decompose_type = 0;
	config.sweep_step = 5;
	config.counter_clockwise = true;
	test.init(config);
	LOG(INFO) << "init success";
	if(argv != 2){
		LOG(INFO) << "the input map is not exist";
		return -1;
	}
	std::string map(argc[1]);
    cv::Mat img = cv::imread(map.c_str());
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat img_ = gray.clone();
    cv::threshold(img_, img_, 250, 255, 0);
    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10), cv::Point(-1,-1)); // size: robot radius
    cv::morphologyEx(img_, img_, cv::MORPH_ERODE, erode_kernel);
    cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5), cv::Point(-1,-1));
    cv::morphologyEx(img_, img_, cv::MORPH_OPEN, open_kernel);
    std::vector<std::vector<cv::Point>> cnts;
    std::vector<cv::Vec4i> hierarchy; // index: next, prev, first_child, parent
    cv::findContours(img_, cnts, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<int> cnt_indices(cnts.size());
    std::iota(cnt_indices.begin(), cnt_indices.end(), 0);
    std::sort(cnt_indices.begin(), cnt_indices.end(), [&cnts](int lhs, int rhs){return cv::contourArea(cnts[lhs]) > cv::contourArea(cnts[rhs]);});
    int ext_cnt_idx = cnt_indices.front();
    std::vector<std::vector<cv::Point>> contours;
    contours.emplace_back(cnts[ext_cnt_idx]);
	for (int i = 0; i < hierarchy.size(); i++) {
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
	std::vector<std::vector<polygon_coverage_planning::Point>> innner_polygon(inner_polys.size());
	for (const auto &point : outer_poly) {
		outer_polygon.push_back(
				polygon_coverage_planning::Point(point.x, point.y));
	}
	for (int i = 0; i < inner_polys.size(); i++) {
		for (const auto &point : inner_polys[i]) {
			innner_polygon[i].push_back(
					polygon_coverage_planning::Point(point.x, point.y));
		}
	}
	test.Update(outer_polygon, innner_polygon);
	LOG(INFO) << "update success";
	std::vector<polygon_coverage_planning::Point> way_points;
	polygon_coverage_planning::Point start = getStartingPoint(img);
	LOG(INFO) << "get start pose success";
	test.SolveDoubleBow(start);
	test.getWayPoint(way_points);
	LOG(INFO) << "solve success";
	// plot for test
	cv::Point p1, p2;
	cv::Mat temp_img = img.clone();
	cv::namedWindow("cover", cv::WINDOW_NORMAL);
	cv::imshow("cover", temp_img);
	// todo add the tag judge for transfer, don't use
	for (size_t i = 1; i < way_points.size(); ++i) {
		if(way_points[i].is_transfer_point || way_points[i-1].is_transfer_point){
			continue;
		}
		p2 = cv::Point(way_points[i].x_, way_points[i].y_);
		p1 = cv::Point(way_points[i - 1].x_, way_points[i - 1].y_);
		cv::line(temp_img, p1, p2, cv::Scalar(0, 64, 255));
		cv::namedWindow("cover", cv::WINDOW_NORMAL);
		cv::imshow("cover", temp_img);
		cv::waitKey(50);
	}
	LOG(INFO)<< "plot success";
	pause();
	return 0;
}
