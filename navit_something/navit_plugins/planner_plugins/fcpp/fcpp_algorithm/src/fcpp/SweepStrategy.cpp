/*
 * SweepStrategy.cpp
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */
#include "fcpp_algorithm/fcpp/SweepStrategy.h"

namespace fcpp {

SweepStrategy::SweepStrategy() {

}

SweepStrategy::~SweepStrategy() {

}

bool SweepStrategy::doReverseNextSweep(const Point_2 &curr_point,
		const std::vector<Point_2> &next_sweep) {
	return CGAL::to_double(
			CGAL::squared_distance(curr_point, next_sweep.front()))
			> CGAL::to_double(
					CGAL::squared_distance(curr_point, next_sweep.back()));
}

SimpleSweepStrategy::SimpleSweepStrategy(){

}

SimpleSweepStrategy::~SimpleSweepStrategy(){

}

bool SimpleSweepStrategy::computeSweep(const Polygon_2 &in,
		const VisibilityGraph &visibility_graph,
		const FT offset, const Direction_2 &dir, bool counter_clockwise,
		std::vector<Point_2> *waypoints) {
	waypoints->clear();
	const FT kSqOffset = offset * offset;
	// Assertions.
	// TODO(rikba): Check monotone perpendicular to dir.
	if (!in.is_counterclockwise_oriented())
		return false;
	// Find start sweep.
	Line_2 sweep(Point_2(0.0, 0.0), dir);
	std::vector<Point_2> sorted_pts = sortVerticesToLine(in, sweep);
	sweep = Line_2(sorted_pts.front(), dir);
	// rotate sweep by 90 degree
	Vector_2 offset_vector =
			sweep.perpendicular(sorted_pts.front()).to_vector();
	offset_vector = offset * offset_vector
			/ std::sqrt(CGAL::to_double(offset_vector.squared_length()));
	const CGAL::Aff_transformation_2<K> kOffset(CGAL::TRANSLATION,
			offset_vector);
	Segment_2 sweep_segment;
	bool has_sweep_segment = findSweepSegment(in, sweep, &sweep_segment);
	while (has_sweep_segment) {
		if (counter_clockwise)
			sweep_segment = sweep_segment.opposite(); // Align sweep segment.
		if (!waypoints->empty()) { // Connect previous sweep
			std::vector<Point_2> shortest_path;
			if (!calculateShortestPath(visibility_graph, waypoints->back(),
					sweep_segment.source(), &shortest_path))
				return false;
			for (std::vector<Point_2>::iterator it = std::next(
					shortest_path.begin());
					it != std::prev(shortest_path.end()); ++it) {
				waypoints->push_back(*it);
			}
		}
		waypoints->push_back(sweep_segment.source()); // Traverse sweep.
		if (!sweep_segment.is_degenerate())
			waypoints->push_back(sweep_segment.target());

		// todo add the circle carve for opposite direction
		sweep = sweep.transform(kOffset);
		Segment_2 prev_sweep_segment =
				counter_clockwise ? sweep_segment.opposite() : sweep_segment; // Find new sweep segment.
		has_sweep_segment = findSweepSegment(in, sweep, &sweep_segment);
		// Add a final sweep.
		// 当前的slice没切到polygon，同时在polygon中的最后一个slice已经遍历过（两个交点），到达终止条件
		if (!has_sweep_segment
				&& !((!waypoints->empty()
						&& *std::prev(waypoints->end(), 1) == sorted_pts.back())
						|| (waypoints->size() > 1
								&& *std::prev(waypoints->end(), 2)
										== sorted_pts.back()))) {
			sweep = Line_2(sorted_pts.back(), dir); // 取polygon中的最后一个slice
			has_sweep_segment = findSweepSegment(in, sweep, &sweep_segment);
			if (!has_sweep_segment) { // 没切到polygon的不要
				DLOG(INFO) << "Failed to calculate final sweep.";
				return false;
			}
			// Do not add super close sweep. sweep之间不能相距太近，太近的不要
			if (CGAL::squared_distance(sweep_segment, prev_sweep_segment) < 0.1)
				break;
		}
		// Check observability of vertices between sweeps.
		// 如果presweep和sweep之间存在距离两个sweep都超过一个offset的slice（无法观测点），则将其补上
		if (has_sweep_segment) {
			std::vector<Point_2>::const_iterator unobservable_point =
					sorted_pts.end();
			checkObservability(prev_sweep_segment, sweep_segment, sorted_pts,
					kSqOffset, &unobservable_point);
			if (unobservable_point != sorted_pts.end()) {
				sweep = Line_2(*unobservable_point, dir);
				has_sweep_segment = findSweepSegment(in, sweep, &sweep_segment);
				if (!has_sweep_segment) {
					DLOG(INFO) << "Failed to calculate extra sweep at point: "
							<< *unobservable_point;
					return false;
				}
			}
		}
		counter_clockwise = !counter_clockwise; // Swap directions.
	}
	return true;
}

void SimpleSweepStrategy::checkObservability(const Segment_2 &prev_sweep, const Segment_2 &sweep,
		const std::vector<Point_2> &sorted_pts, const FT max_sq_distance,
		std::vector<Point_2>::const_iterator *lowest_unobservable_point) {
	*lowest_unobservable_point = sorted_pts.end();
	// Find first point that is between prev_sweep and sweep and unobservable.
	for (std::vector<Point_2>::const_iterator it = sorted_pts.begin();
			it != sorted_pts.end(); ++it) {
		// 左边/逆时针 = 正； 右边/顺时针 = 负
		if (prev_sweep.supporting_line().has_on_positive_side(*it))
			continue;
		if (sweep.supporting_line().has_on_negative_side(*it)) {
			break;
		}
		// 在sweep和pre-sweep之间才计算，要是没有符合条件的，it就是初始化状态，指向polygon的最后一个slice
		FT sq_distance_prev = CGAL::squared_distance(prev_sweep, *it);
		FT sq_distance_curr = CGAL::squared_distance(sweep, *it);
		if (sq_distance_prev > max_sq_distance
				&& sq_distance_curr > max_sq_distance) {
			*lowest_unobservable_point = it;
			return;
		}
	}
}



std::vector<Point_2> SimpleSweepStrategy::findIntersections(const Polygon_2 &p, const Line_2 &l) {
	std::vector<Point_2> intersections;
	typedef CGAL::cpp11::result_of<Intersect_2(Segment_2, Line_2)>::type Intersection;
	for (EdgeConstIterator it = p.edges_begin(); it != p.edges_end(); ++it) {
		Intersection result = CGAL::intersection(*it, l);
		if (result) {
			if (const Segment_2 *s = boost::get<Segment_2>(&*result)) {
				intersections.push_back(s->source());
				intersections.push_back(s->target());
			} else {
				intersections.push_back(*boost::get<Point_2>(&*result));
			}
		}
	}
	Line_2 perp_l = l.perpendicular(l.point(0)); // Sort.
	std::sort(intersections.begin(), intersections.end(),
			[&perp_l](const Point_2 &a, const Point_2 &b) -> bool {
				return CGAL::has_smaller_signed_distance_to_line(perp_l, a, b);
			});
	return intersections;
}

bool SimpleSweepStrategy::findSweepSegment(const Polygon_2 &p, const Line_2 &l,
		Segment_2 *sweep_segment) {
	std::vector<Point_2> intersections = findIntersections(p, l);
	if (intersections.empty())
		return false;
	*sweep_segment = Segment_2(intersections.front(), intersections.back());
	return true;
}

std::vector<Point_2> SimpleSweepStrategy::sortVerticesToLine(const Polygon_2 &p, const Line_2 &l) {
	std::vector<Point_2> pts(p.size()); // Copy points.
	std::vector<Point_2>::iterator pts_it = pts.begin();
	for (VertexConstIterator it = p.vertices_begin(); it != p.vertices_end();
			++it) {
		*(pts_it++) = *it;
	}
	std::sort(pts.begin(), pts.end(),
			[&l](const Point_2 &a, const Point_2 &b) -> bool {
				return CGAL::has_smaller_signed_distance_to_line(l, a, b);
			});
	return pts;
}

bool SimpleSweepStrategy::calculateShortestPath(
		const VisibilityGraph &visibility_graph,
		const Point_2 &start, const Point_2 &goal,
		std::vector<Point_2> *shortest_path) {
	shortest_path->clear();
	Polygon_2 start_visibility, goal_visibility;
	if (!common::computeVisibilityPolygon(visibility_graph.getPolygon(), start,
			&start_visibility)) {
		DLOG(INFO) << "Cannot compute visibility polygon from start query point "
				<< start << " in polygon: " << visibility_graph.getPolygon();
		return false;
	}
	if (!common::computeVisibilityPolygon(visibility_graph.getPolygon(), goal,
			&goal_visibility)) {
		DLOG(INFO) << "Cannot compute visibility polygon from goal query point "
				<< goal << " in polygon: " << visibility_graph.getPolygon();
		return false;
	}
	if (!visibility_graph.solve(start, start_visibility, goal, goal_visibility,
			shortest_path)) {
		DLOG(INFO) << "Cannot compute shortest path from " << start << " to "
				<< goal << " in polygon: " << visibility_graph.getPolygon();
		return false;
	}
	if (shortest_path->size() < 2) {
		DLOG(INFO) << "Shortest path too short.";
		return false;
	}
	return true;
}


} // namespace fcpp
