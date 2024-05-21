/*
 * CoveragePlanner.h
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#ifndef FCPP_ALGORITHM_INCLUDE_COVERAGEPLANNER_H_
#define FCPP_ALGORITHM_INCLUDE_COVERAGEPLANNER_H_

#include "fcpp_algorithm/fcpp/SweepStrategy.h"
#include "fcpp_algorithm/fcpp/VisibilityGraph.h"
#include "fcpp_algorithm/fcpp/TravellingSolver.h"
#include "fcpp_algorithm/fcpp/DomainDecompose.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <mutex>
namespace polygon_coverage_planning {

struct CoveragePlannerConfig {
	int decompose_type;
	int tsp_type;
	float sweep_step;
	float robot_radius; // for counter compute
	bool counter_clockwise; //
};

struct Point {
	Point(float x, float y, bool is_transfer = false) :
			x_(x), y_(y), is_transfer_point(is_transfer) {
	}
	Point() {
		x_ = -1;
		y_ = -1;
		is_transfer_point = false;
	}
	float x_;
	float y_;
	bool is_transfer_point;
};

class CoveragePlanner {
public:
	CoveragePlanner();
	virtual ~CoveragePlanner();
	// this API support in other thread init
	bool init(const CoveragePlannerConfig &config);
	// update the polygon
	bool Update(const std::vector<Point> &outer_poly,
			const std::vector<std::vector<Point>> &inner_poly);
	// solve and get the sweep path
	bool SolveSingleBow(const Point start);

	// solve and get the sweep path
	bool SolveDoubleBow(const Point start);

	// todo add the judge
	void getWayPoint(std::vector<Point> &waypoint);
private:
	std::mutex config_mutex;
	CoveragePlannerConfig config_;
	std::shared_ptr<PolygonWithHoles> polygon_;
	std::shared_ptr<fcpp::TravellingSolver> tsp_solver_;
	std::shared_ptr<fcpp::DomainDecompose> decompose_;
	std::shared_ptr<fcpp::SweepStrategy> sweep_;
	struct WaypointReult{
	public:
		WaypointReult(Point_2 p, bool transfer):
			point(p),is_transfer(transfer){

		};
		WaypointReult(){
		};
		Point_2 point;
		bool is_transfer;
	};
	std::vector<WaypointReult> way_points;
};

} //namespace polygon_coverage_planning

#endif /* FCPP_ALGORITHM_INCLUDE_COVERAGEPLANNER_H_ */
