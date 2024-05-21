/*
 * VisibilityGraph.h
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#ifndef FCPP_ALGORITHM_INCLUDE_VISIBILITYGRAPH_H_
#define FCPP_ALGORITHM_INCLUDE_VISIBILITYGRAPH_H_

#include "fcpp_algorithm/common/def.h"
#include "fcpp_algorithm/common/graph_base.h"
#include "fcpp_algorithm/common/utils.h"

namespace fcpp {

struct NodeProperty {
	NodeProperty() :
			coordinates(Point_2(CGAL::ORIGIN)) {
	}
	NodeProperty(const Point_2 &coordinates, const Polygon_2 &visibility) :
			coordinates(coordinates), visibility(visibility) {
	}
	Point_2 coordinates;   // The 2D coordinates.
	Polygon_2 visibility;  // The visibile polygon from the vertex.
};

struct EdgeProperty {
};

class VisibilityGraph: public common::GraphBase<NodeProperty, EdgeProperty> {
public:
	VisibilityGraph(const PolygonWithHoles &polygon);
	VisibilityGraph(const Polygon_2 &polygon) :
			VisibilityGraph(PolygonWithHoles(polygon)) {
	}
	VisibilityGraph(): GraphBase() {
	}
	virtual ~VisibilityGraph();

	virtual bool create() override;
	// Compute the shortest path in a polygon with holes using A* and the
	// precomputed visibility graph.
	// If start or goal are outside the polygon, they are snapped (projected) back
	// into it.
	bool solve(const Point_2 &start, const Point_2 &goal,
			std::vector<Point_2> *waypoints) const;
	// Same as solve but provide a precomputed visibility graph for the polygon.
	// Note: Start and goal need to be contained in the polygon_.
	bool solve(const Point_2 &start, const Polygon_2 &start_visibility_polygon,
			const Point_2 &goal, const Polygon_2 &goal_visibility_polygon,
			std::vector<Point_2> *waypoints) const;

	// Convenience function: addtionally adds original start and goal to shortest
	// path, if they were outside of polygon.
	bool solveWithOutsideStartAndGoal(const Point_2 &start, const Point_2 &goal,
			std::vector<Point_2> *waypoints) const;
	// Given a solution, get the concatenated 2D waypoints.
	bool getWaypoints(const common::Solution &solution,
			std::vector<Point_2> *waypoints) const;

	PolygonWithHoles getPolygon() const ;
private:
	// Sort boundary to be counter-clockwise and holes to be clockwise.
	void sortVertices();

	bool pointsInPolygon(
			const std::vector<Point_2>::iterator &begin,
			const std::vector<Point_2>::iterator &end);

	// Project a point on a polygon.
	Point_2 projectOnPolygon2(const Polygon_2 &poly, const Point_2 &p,
			FT *squared_distance) const ;

	// Project a point on the polygon boundary.
	Point_2 projectPointOnHull(const Point_2 &p) const;

	// Adds all line of sight neighbors.The graph is acyclic
	// and undirected and thus forms a symmetric adjacency matrix.
	virtual bool addEdges() override;

	// Calculate the Euclidean distance to goal for all given nodes.
	virtual bool calculateHeuristic(size_t goal, common::Heuristic *heuristic) const
			override;

	// Find and append concave outer boundary vertices.
	void findConcaveOuterBoundaryVertices(
			std::vector<VertexConstCirculator> *concave_vertices) const;
	// Find and append convex hole vertices.
	void findConvexHoleVertices(
			std::vector<VertexConstCirculator> *convex_vertices) const;

	// Given two waypoints, compute its euclidean distance.
	double computeEuclideanSegmentCost(const Point_2 &from,
			const Point_2 &to) const;
private:
	PolygonWithHoles polygon_;
};

} // namespace polygon_coverage_planning
#endif /* FCPP_ALGORITHM_INCLUDE_VISIBILITYGRAPH_H_ */
