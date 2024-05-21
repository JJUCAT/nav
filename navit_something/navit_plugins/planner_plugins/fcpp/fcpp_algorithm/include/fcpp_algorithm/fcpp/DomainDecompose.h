/*
 * DomainDecompose.h
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#ifndef FCPP_ALGORITHM_INCLUDE_DOMAINDECOMPOSE_H_
#define FCPP_ALGORITHM_INCLUDE_DOMAINDECOMPOSE_H_

#include "fcpp_algorithm/common/def.h"

namespace fcpp {

class DomainDecompose {
public:
	DomainDecompose();
	virtual ~DomainDecompose();
	void setPolygonWithHoles(const PolygonWithHoles &polygon_in);
	virtual bool compute(std::vector<Polygon_2> &bcd_polygons) = 0;
	virtual bool compute(std::vector<Polygon_2> &bcd_polygons, const Direction_2 dir) = 0;
	double findBestSweepDir(const Polygon_2 &cell, Direction_2 *best_dir =
			nullptr);
	// Get all directions that are perpendicular to the edges found with findEdgeDirections.
	int getCellIndexOfPoint(const std::vector<Polygon_2> &decompositions,
			const Point_2 &point);

	// get the best decompose direction
	bool getBestDecomposeDir(Direction_2 & dir);
protected:
	std::vector<Direction_2> findPerpEdgeDirections(const PolygonWithHoles &pwh);
	// Get all unique polygon edge directions including opposite directions.
	std::vector<Direction_2> findEdgeDirections(const PolygonWithHoles &pwh);

	// Check whether polygon 'in' is weakly monotone perpendicular to 'x_axis'.
	bool isWeaklyMonotone(const Polygon_2 &in, const Line_2 &x_axis);
	// For all edges check whether polygon 'in' is weakly monotone perpendicular to that edge.
	std::vector<Direction_2> getAllSweepableEdgeDirections(const Polygon_2 &in);

	// todo add other direction
	VertexConstCirculator findSouth(const Polygon_2 &in, const Line_2 &x_axis);
	VertexConstCirculator findNorth(const Polygon_2 &in, const Line_2 &x_axis);
protected:
	PolygonWithHoles polyon_;
	std::shared_ptr<Direction_2> best_dir_;
};

class BCD: public DomainDecompose {
public:
	BCD();
	bool compute(std::vector<Polygon_2> &bcd_polygons) override;
	bool compute(std::vector<Polygon_2> &bcd_polygons, const Direction_2 dir) override;
	virtual ~BCD();
private:
	std::vector<Polygon_2> findBcdOnDir(const Direction_2 &dir);

	std::vector<VertexConstCirculator> getXSortedVertices(
			const PolygonWithHoles &p);
	void processEvent(const PolygonWithHoles &pwh,
			const VertexConstCirculator &v,
			std::vector<VertexConstCirculator> *sorted_vertices,
			std::vector<Point_2> *processed_vertices, std::list<Segment_2> *L,
			std::list<Polygon_2> *open_polygons,
			std::vector<Polygon_2> *closed_polygons);
	std::vector<Point_2> getIntersections(const std::list<Segment_2> &L,
			const Line_2 &l);
	bool outOfPWH(const PolygonWithHoles &pwh, const Point_2 &p);

	void sortPolygon(PolygonWithHoles *pwh);
	// Removes duplicate vertices. Returns if resulting polygon is simple and has some area.
	bool cleanupPolygon(Polygon_2 *poly);
};

class TCD: public DomainDecompose {
public:
	TCD();
	virtual ~TCD();
	bool compute(std::vector<Polygon_2> &bcd_polygons) override;
	bool compute(std::vector<Polygon_2> &bcd_polygons, const Direction_2 dir) override;
private:
	std::vector<Polygon_2> findTcdOnDir(const Direction_2 &dir);
};

} // namespace fcpp

#endif /* FCPP_ALGORITHM_INCLUDE_DOMAINDECOMPOSE_H_ */
