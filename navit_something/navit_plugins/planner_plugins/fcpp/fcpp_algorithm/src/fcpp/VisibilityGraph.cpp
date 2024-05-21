/*
 * VisibilityGraph.cpp
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#include "fcpp_algorithm/fcpp/VisibilityGraph.h"

namespace fcpp {

VisibilityGraph::~VisibilityGraph() {
	// TODO Auto-generated destructor stub
}

VisibilityGraph::VisibilityGraph(const PolygonWithHoles &polygon) :
		GraphBase(), polygon_(polygon) {
	is_created_ = create(); // Build visibility graph.
}

bool VisibilityGraph::create() {
	clear();
	sortVertices(); // Sort vertices.
	std::vector<VertexConstCirculator> graph_vertices; // Select shortest path vertices.
	findConcaveOuterBoundaryVertices(&graph_vertices);
	findConvexHoleVertices(&graph_vertices);
	for (const VertexConstCirculator &v : graph_vertices) {
		Polygon_2 visibility; // Compute visibility polygon.
		if (!common::computeVisibilityPolygon(polygon_, *v, &visibility)) {
			DLOG(INFO) << "Cannot compute visibility polygon.";
			return false;
		}
		if (!addNode(NodeProperty(*v, visibility))) {
			return false;
		}
	}
	return true;
}

// create the outer boundary(filter the un-concave)
void VisibilityGraph::findConcaveOuterBoundaryVertices(
		std::vector<VertexConstCirculator> *concave_vertices) const {
	VertexConstCirculator vit = polygon_.outer_boundary().vertices_circulator();
	do {
		Triangle_2 triangle(*std::prev(vit), *vit, *std::next(vit));
		CGAL::Orientation orientation = triangle.orientation();
		if (orientation == CGAL::CLOCKWISE)
			concave_vertices->push_back(vit); //for Concave check
	} while (++vit != polygon_.outer_boundary().vertices_circulator());
}

// create the hole boundary(filter the un-concave)
void VisibilityGraph::findConvexHoleVertices(
		std::vector<VertexConstCirculator> *convex_vertices) const {
	for (PolygonWithHoles::Hole_const_iterator hit = polygon_.holes_begin();
			hit != polygon_.holes_end(); ++hit) {
		VertexConstCirculator vit = hit->vertices_circulator();
		do {
			Triangle_2 triangle(*std::prev(vit), *vit, *std::next(vit));
			CGAL::Orientation orientation = triangle.orientation();
			if (orientation == CGAL::CLOCKWISE)
				convex_vertices->push_back(vit);
		} while (++vit != hit->vertices_circulator());
	}
}

bool VisibilityGraph::addEdges() {
	if (graph_.empty()) {
		DLOG(INFO) << "Cannot add edges to an empty graph." ;
		return false;
	}
	const size_t new_id = graph_.size() - 1;
	for (size_t adj_id = 0; adj_id < new_id; ++adj_id) {
		const NodeProperty *new_node_property = getNodeProperty(new_id);
		const NodeProperty *adj_node_property = getNodeProperty(adj_id);
		if (adj_node_property == nullptr) {
			DLOG(INFO) << "Cannot access potential neighbor." ;
			return false;
		}

		if (common::pointInPolygon(new_node_property->visibility,
				adj_node_property->coordinates)) {
			common::EdgeId forwards_edge_id(new_id, adj_id);
			common::EdgeId backwards_edge_id(adj_id, new_id);
			const double cost = computeEuclideanSegmentCost(
					new_node_property->coordinates,
					adj_node_property->coordinates);  // Symmetric cost.
			if (!addEdge(forwards_edge_id, EdgeProperty(), cost)
					|| !addEdge(backwards_edge_id, EdgeProperty(), cost)) {
				return false;
			}
		}
	}
	return true;
}

bool VisibilityGraph::solve(const Point_2 &start, const Point_2 &goal,
		std::vector<Point_2> *waypoints) const {
	waypoints->clear(); // Make sure start and end are inside the polygon.
	const Point_2 start_new =
			common::pointInPolygon(polygon_, start) ?
					start : projectPointOnHull(start);
	const Point_2 goal_new =
			common::pointInPolygon(polygon_, goal) ?
					goal : projectPointOnHull(goal);
	// Compute start and goal visibility polygon.
	Polygon_2 start_visibility, goal_visibility;
	if (!common::computeVisibilityPolygon(polygon_, start_new, &start_visibility)
			|| !common::computeVisibilityPolygon(polygon_, goal_new,
					&goal_visibility)) {
		return false;
	}
	return solve(start_new, start_visibility, goal_new, goal_visibility,
			waypoints);
}

Point_2 VisibilityGraph::projectPointOnHull(const Point_2 &p) const {
	// Project point on outer boundary.
	FT min_distance;
	Point_2 projection = projectOnPolygon2(polygon_.outer_boundary(), p,
			&min_distance);
	// Project on holes.
	for (PolygonWithHoles::Hole_const_iterator hit = polygon_.holes_begin();
			hit != polygon_.holes_end(); ++hit) {
		FT temp_distance;
		Point_2 temp_projection = projectOnPolygon2(*hit, p, &temp_distance);
		if (temp_distance < min_distance) {
			min_distance = temp_distance;
			projection = temp_projection;
		}
	}
	return projection;
}

Point_2 VisibilityGraph::projectOnPolygon2(const Polygon_2 &poly,
		const Point_2 &p, FT *squared_distance) const {
	std::vector<std::pair<FT, EdgeConstIterator>> edge_distances(poly.size()); // Find the closest edge.
	std::vector<std::pair<FT, EdgeConstIterator>>::iterator dit =
			edge_distances.begin();
	for (EdgeConstIterator eit = poly.edges_begin(); eit != poly.edges_end();
			eit++, dit++) {
		dit->first = CGAL::squared_distance(*eit, p);
		dit->second = eit;
	}
	std::vector<std::pair<FT, EdgeConstIterator>>::iterator closest_pair =
			std::min_element(edge_distances.begin(), edge_distances.end(),
					[](const std::pair<FT, EdgeConstIterator> &lhs,
							const std::pair<FT, EdgeConstIterator> &rhs) {
						return lhs.first < rhs.first;
					});
	EdgeConstIterator closest_edge = closest_pair->second;
	*squared_distance = closest_pair->first;
	// Project p on supporting line of closest edge.
	Point_2 projection = closest_edge->supporting_line().projection(p);
	// Check if p is on edge. If not snap it to source or target.
	if (!closest_edge->has_on(projection)) {
		FT d_source = CGAL::squared_distance(p, closest_edge->source());
		FT d_target = CGAL::squared_distance(p, closest_edge->target());
		projection =
				d_source < d_target ?
						closest_edge->source() : closest_edge->target();
	}
	return projection;
}

bool VisibilityGraph::solve(const Point_2 &start,
		const Polygon_2 &start_visibility_polygon, const Point_2 &goal,
		const Polygon_2 &goal_visibility_polygon,
		std::vector<Point_2> *waypoints) const {
	waypoints->clear();
	if (!is_created_) {
		DLOG(INFO) << "Visibility graph not initialized." ;
		return false;
	} else if (!common::pointInPolygon(polygon_, start)
			|| !common::pointInPolygon(polygon_, goal)) {
		DLOG(INFO) << "Start or goal is not in polygon." ;
		return false;
	}
	VisibilityGraph temp_visibility_graph = *this;
	// Add start and goal node.
	if (!temp_visibility_graph.addStartNode(
			NodeProperty(start, start_visibility_polygon))
			|| !temp_visibility_graph.addGoalNode(
					NodeProperty(goal, goal_visibility_polygon))) {
		return false;
	}
	// Check if start and goal are in line of sight.
	size_t start_idx = temp_visibility_graph.getStartIdx();
	size_t goal_idx = temp_visibility_graph.getGoalIdx();
	const NodeProperty *start_node_property =
			temp_visibility_graph.getNodeProperty(start_idx);
	if (start_node_property == nullptr) {
		return false;
	}
	// this api have problem
	if (common::pointInPolygon(start_node_property->visibility, goal)) {
		waypoints->push_back(start);
		waypoints->push_back(goal);
		return true;
	}
	// Find shortest way using A*.
	common::Solution solution;
	if (!temp_visibility_graph.solveAStar(start_idx, goal_idx, &solution)) {
		DLOG(INFO) << "Could not find shortest path. Graph not fully connected.";
		return false;
	}
	// Reconstruct waypoints.
	return temp_visibility_graph.getWaypoints(solution, waypoints);
}

bool VisibilityGraph::getWaypoints(const common::Solution &solution,
		std::vector<Point_2> *waypoints) const {
	waypoints->resize(solution.size());
	for (size_t i = 0; i < solution.size(); i++) {
		const NodeProperty *node_property = getNodeProperty(solution[i]);
		if (node_property == nullptr) {
			DLOG(INFO) << "Cannot reconstruct solution.";
			return false;
		}
		(*waypoints)[i] = node_property->coordinates;
	}
	return true;
}

PolygonWithHoles VisibilityGraph::getPolygon() const {
	return polygon_;
}

bool VisibilityGraph::calculateHeuristic(size_t goal,
		common::Heuristic *heuristic) const {
	heuristic->clear();
	const NodeProperty *goal_node_property = getNodeProperty(goal);
	if (goal_node_property == nullptr) {
		DLOG(INFO) << "Cannot find goal node property to calculate heuristic.";
		return false;
	}
	for (size_t adj_id = 0; adj_id < graph_.size(); ++adj_id) {
		const NodeProperty *adj_node_property = getNodeProperty(adj_id);
		if (adj_node_property == nullptr) {
			DLOG(INFO) << "Cannot access adjacent node property to calculate heuristic.";
			return false;
		}
		(*heuristic)[adj_id] = computeEuclideanSegmentCost(
				adj_node_property->coordinates,
				goal_node_property->coordinates);
	}
	return true;
}

bool VisibilityGraph::solveWithOutsideStartAndGoal(const Point_2 &start,
		const Point_2 &goal, std::vector<Point_2> *waypoints) const {
	if (solve(start, goal, waypoints)) {
		if (!common::pointInPolygon(polygon_, start)) {
			waypoints->insert(waypoints->begin(), start);
		}
		if (!common::pointInPolygon(polygon_, goal)) {
			waypoints->push_back(goal);
		}
		return true;
	} else {
		return false;
	}
}

void VisibilityGraph::sortVertices() {
	if (polygon_.outer_boundary().is_clockwise_oriented())
		polygon_.outer_boundary().reverse_orientation();

	for (PolygonWithHoles::Hole_iterator hi = polygon_.holes_begin();
			hi != polygon_.holes_end(); ++hi)
		if (hi->is_counterclockwise_oriented())
			hi->reverse_orientation();
}

double VisibilityGraph::computeEuclideanSegmentCost(const Point_2 &from,
		const Point_2 &to) const {
	return std::sqrt(CGAL::to_double(Segment_2(from, to).squared_length()));
}

bool VisibilityGraph::pointsInPolygon(
		const std::vector<Point_2>::iterator &begin,
		const std::vector<Point_2>::iterator &end) {
	for (std::vector<Point_2>::iterator it = begin; it != end; ++it) {
		if (!common::pointInPolygon(polygon_, *it))
			return false;
	}
	return true;
}

}
