/*
 * SweepStrategy.h
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#ifndef FCPP_ALGORITHM_INCLUDE_SWEEPSTRATEGY_H_
#define FCPP_ALGORITHM_INCLUDE_SWEEPSTRATEGY_H_

#include "fcpp_algorithm/fcpp/VisibilityGraph.h"
#include "fcpp_algorithm/common/def.h"

namespace fcpp {

class SweepStrategy {
public:
	SweepStrategy();
	virtual ~SweepStrategy();
	virtual bool computeSweep(const Polygon_2 &in,
			const VisibilityGraph &visibility_graph,
			const FT offset, const Direction_2 &dir, bool counter_clockwise,
			std::vector<Point_2> *waypoints) = 0;

	bool doReverseNextSweep(const Point_2 &curr_point,
			const std::vector<Point_2> &next_sweep);
};

class SimpleSweepStrategy: public SweepStrategy{
public:
	SimpleSweepStrategy();
	virtual ~SimpleSweepStrategy();
	bool computeSweep(const Polygon_2 &in,
				const VisibilityGraph &visibility_graph,
				const FT offset, const Direction_2 &dir, bool counter_clockwise,
				std::vector<Point_2> *waypoints) override;
private:
	// A segment is observable if all vertices between two sweeps are observable.
	void checkObservability(
	    const Segment_2& prev_sweep, const Segment_2& sweep,
	    const std::vector<Point_2>& sorted_pts, const FT max_sq_distance,
	    std::vector<Point_2>::const_iterator* lowest_unobservable_point);

	// Find the intersections between a polygon and a line and sort them by the
	// distance to the perpendicular direction of the line.
	std::vector<Point_2> findIntersections(const Polygon_2& p, const Line_2& l);

	// Same as findIntersections but only return first and last intersection.
	bool findSweepSegment(const Polygon_2& p, const Line_2& l,
	                      Segment_2* sweep_segment);

	// Sort vertices of polygon based on signed distance to line l.
	std::vector<Point_2> sortVerticesToLine(const Polygon_2& p, const Line_2& l);

	// Connect to points in the polygon using the visibility graph.
	bool calculateShortestPath(
	    const VisibilityGraph& visibility_graph,
	    const Point_2& start, const Point_2& goal,
	    std::vector<Point_2>* shortest_path);
};

} // namespace polygon_coverage_planning

#endif /* FCPP_ALGORITHM_INCLUDE_SWEEPSTRATEGY_H_ */
