/*
 * utils.h
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#ifndef FCPP_ALGORITHM_COMMON_UTILS_H_
#define FCPP_ALGORITHM_COMMON_UTILS_H_

#include "fcpp_algorithm/common/def.h"

namespace common {
// Helper to check whether a point is inside or on the boundary of the polygon.
bool pointInPolygon(const PolygonWithHoles &pwh, const Point_2 &p);

bool pointInPolygon(const Polygon_2& poly, const Point_2& p);

// Definition according to
// https://doc.cgal.org/latest/Straight_skeleton_2/index.html
bool isStrictlySimple(const PolygonWithHoles &pwh);

// get the area avoid of holes
FT computeArea(const PolygonWithHoles &pwh);
FT computeArea(const Polygon_2 &poly);

// Remove col-linear vertices.
void simplifyPolygon(Polygon_2 *polygon);
void simplifyPolygon(PolygonWithHoles *pwh);

// rotate the polygon
PolygonWithHoles rotatePolygon(const PolygonWithHoles &polygon_in,
		const Direction_2 &dir);

// get the point of boundary
std::vector<Point_2> getHullVertices(const PolygonWithHoles &pwh);
std::vector<std::vector<Point_2>> getHoleVertices(const PolygonWithHoles &pwh);

// Compute the visibility polygon given a point inside a strictly simple
// polygon. Francisc Bungiu, Michael Hemmer, John Hershberger, Kan Huang, and
// Alexander Kröller. Efficient computation of visibility polygons. CoRR,
// abs/1403.3905, 2014.
bool computeVisibilityPolygon(const PolygonWithHoles &pwh,
		const Point_2 &query_point, Polygon_2 *visibility_polygon);

} //namespace common

#endif /* FCPP_ALGORITHM_COMMON_UTILS_H_ */
