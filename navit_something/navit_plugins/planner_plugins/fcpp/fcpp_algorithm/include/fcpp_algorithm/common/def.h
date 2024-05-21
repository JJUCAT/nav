/*
 * def.h
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#ifndef FCPP_ALGORITHM_COMMON_DEF_H_
#define FCPP_ALGORITHM_COMMON_DEF_H_

//GLOG
#include "glog/logging.h"

// CGAL
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Polygon_vertical_decomposition_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Surface_sweep_2_algorithms.h>
#include <CGAL/squared_distance_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;
typedef K::Vector_2 Vector_2;
typedef K::Direction_2 Direction_2;
typedef K::Line_2 Line_2;
typedef K::Intersect_2 Intersect_2;
typedef K::Plane_3 Plane_3;
typedef K::Segment_2 Segment_2;
typedef K::Triangle_2 Triangle_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef Polygon_2::Vertex_const_iterator VertexConstIterator;
typedef Polygon_2::Vertex_const_circulator VertexConstCirculator;
typedef Polygon_2::Vertex_iterator VertexIterator;
typedef Polygon_2::Vertex_circulator VertexCirculator;
typedef Polygon_2::Edge_const_iterator EdgeConstIterator;
typedef Polygon_2::Edge_const_circulator EdgeConstCirculator;
typedef CGAL::Polygon_with_holes_2<K> PolygonWithHoles;
typedef CGAL::Exact_predicates_inexact_constructions_kernel InexactKernel;

#endif /* FCPP_ALGORITHM_COMMON_DEF_H_ */
