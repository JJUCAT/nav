/*
 * utils.cpp
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#include "fcpp_algorithm/common/utils.h"
#include "fcpp_algorithm/common/def.h"

namespace common {

bool pointInPolygon(const PolygonWithHoles& pwh, const Point_2& p) {
	// Point inside outer boundary.
	CGAL::Bounded_side result = CGAL::bounded_side_2(
			pwh.outer_boundary().vertices_begin(),
			pwh.outer_boundary().vertices_end(), p, K());
	if (result == CGAL::ON_UNBOUNDED_SIDE)
		return false;
	// Point outside hole.
	for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
			hit != pwh.holes_end(); ++hit) {
		result = CGAL::bounded_side_2(hit->vertices_begin(),
				hit->vertices_end(), p, K());
		if (result == CGAL::ON_BOUNDED_SIDE)
			return false;
	}
	return true;
}

bool pointInPolygon(const Polygon_2& poly, const Point_2& p)
{
 return pointInPolygon(PolygonWithHoles(poly), p);
}

bool isStrictlySimple(const PolygonWithHoles &pwh) {
	for (PolygonWithHoles::Hole_const_iterator hi = pwh.holes_begin();
			hi != pwh.holes_end(); ++hi)
		if (!hi->is_simple())
			return false;
	return pwh.outer_boundary().is_simple();
}

FT computeArea(const PolygonWithHoles &pwh) {
	FT area = CGAL::abs(
			CGAL::polygon_area_2(pwh.outer_boundary().vertices_begin(),
					pwh.outer_boundary().vertices_end(), K()));
	for (PolygonWithHoles::Hole_const_iterator hi = pwh.holes_begin();
			hi != pwh.holes_end(); ++hi)
		area -= CGAL::abs(
				CGAL::polygon_area_2(hi->vertices_begin(), hi->vertices_end(),
						K()));
	return area;
}


FT computeArea(const Polygon_2 &poly) {
	return computeArea(PolygonWithHoles(poly));
}

void simplifyPolygon(Polygon_2 *polygon) {
	std::vector<Polygon_2::Vertex_circulator> v_to_erase;
	Polygon_2::Vertex_circulator vc = polygon->vertices_circulator();
	// Find collinear vertices.
	for(auto it = polygon->begin();it!= polygon->end();++it){
		DLOG(INFO)<< "cur pos is (" << CGAL::to_double(it->x()) <<"," << CGAL::to_double(it->y())<< ")" ;
	}
	do {
		if (CGAL::collinear(*std::prev(vc), *vc, *std::next(vc))) {
			v_to_erase.push_back(vc);
		}
	} while (++vc != polygon->vertices_circulator());
	// Remove intermediate vertices.
	for (std::vector<Polygon_2::Vertex_circulator>::reverse_iterator rit =
			v_to_erase.rbegin(); rit != v_to_erase.rend(); ++rit) {
		polygon->erase(*rit);
	}
}

void simplifyPolygon(PolygonWithHoles *pwh) {
	static int count = 0;
	count++;
	DLOG(INFO)<< "the counts is: "<< count;
	DLOG(INFO) << "start simplify outer boundary";
	simplifyPolygon(&pwh->outer_boundary());
	DLOG(INFO)<< "start simplify the holes";
	for (PolygonWithHoles::Hole_iterator hi = pwh->holes_begin();
			hi != pwh->holes_end(); ++hi){
		simplifyPolygon(&*hi);
	}
}

PolygonWithHoles rotatePolygon(const PolygonWithHoles &polygon_in,
		const Direction_2 &dir) {
	CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
	rotation = rotation.inverse();
	PolygonWithHoles rotated_polygon = polygon_in;
	rotated_polygon.outer_boundary() = CGAL::transform(rotation,
			polygon_in.outer_boundary());
	PolygonWithHoles::Hole_iterator hit_rot = rotated_polygon.holes_begin();
	for (PolygonWithHoles::Hole_const_iterator hit = polygon_in.holes_begin();
			hit != polygon_in.holes_end(); ++hit) {
		*(hit_rot++) = CGAL::transform(rotation, *hit);
	}
	return rotated_polygon;
}

std::vector<Point_2> getHullVertices(const PolygonWithHoles &pwh) {
	std::vector<Point_2> vec(pwh.outer_boundary().size());
	std::vector<Point_2>::iterator vecit = vec.begin();
	for (VertexConstIterator vit = pwh.outer_boundary().vertices_begin();
			vit != pwh.outer_boundary().vertices_end(); ++vit, ++vecit)
		*vecit = *vit;
	return vec;
}

std::vector<std::vector<Point_2>> getHoleVertices(const PolygonWithHoles &pwh) {
	std::vector<std::vector<Point_2>> hole_vertices(pwh.number_of_holes());
	std::vector<std::vector<Point_2>>::iterator hvit = hole_vertices.begin();
	for (PolygonWithHoles::Hole_const_iterator hi = pwh.holes_begin();
			hi != pwh.holes_end(); ++hi, ++hvit) {
		hvit->resize(hi->size());
		std::vector<Point_2>::iterator it = hvit->begin();
		for (VertexConstIterator vit = hi->vertices_begin();
				vit != hi->vertices_end(); ++vit, ++it)
			*it = *vit;
	}
	return hole_vertices;
}

bool computeVisibilityPolygon(const PolygonWithHoles& pwh, const Point_2 &query_point,
		Polygon_2 *visibility_polygon) {
	// Preconditions.
	if (!pointInPolygon(pwh,query_point)) {
		std::cout << "Query point outside of polygon." << std::endl;
	}
	if (!isStrictlySimple(pwh)) {
		std::cout << "Polygon is not strictly simple." << std::endl;
	}
	// Create 2D arrangement.
	typedef CGAL::Arr_segment_traits_2<K> VisibilityTraits;
	typedef CGAL::Arrangement_2<VisibilityTraits> VisibilityArrangement;
	VisibilityArrangement poly;
	CGAL::insert(poly, pwh.outer_boundary().edges_begin(),
			pwh.outer_boundary().edges_end());
	// Store main face.
	VisibilityArrangement::Face_const_handle main_face = poly.faces_begin();
	while (main_face->is_unbounded()) {
		main_face++;
	}
	for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
			hit != pwh.holes_end(); ++hit)
		CGAL::insert(poly, hit->edges_begin(), hit->edges_end());

	// Create Triangular Expansion Visibility object.
	typedef CGAL::Triangular_expansion_visibility_2<VisibilityArrangement,
			CGAL::Tag_true> TEV;
	TEV tev(poly);
	// We need to determine the halfedge or face to which the query point
	// corresponds.
	typedef CGAL::Arr_naive_point_location<VisibilityArrangement> NaivePL;
	typedef CGAL::Arr_point_location_result<VisibilityArrangement>::Type PLResult;
	NaivePL pl(poly);
	PLResult pl_result = pl.locate(query_point);
	VisibilityArrangement::Vertex_const_handle *v = nullptr;
	VisibilityArrangement::Halfedge_const_handle *e = nullptr;
	VisibilityArrangement::Face_const_handle *f = nullptr;
	typedef VisibilityArrangement::Face_handle VisibilityFaceHandle;
	VisibilityFaceHandle fh;
	VisibilityArrangement visibility_arr;
	if ((f = boost::get<VisibilityArrangement::Face_const_handle>(&pl_result))) {
		fh = tev.compute_visibility(query_point, *f, visibility_arr); // Located in face.
	} else if ((v = boost::get<VisibilityArrangement::Vertex_const_handle>(
			&pl_result))) {
		// Located on vertex. Search the incident halfedge that contains the polygon face.
		VisibilityArrangement::Halfedge_const_handle he =
				poly.halfedges_begin();
		while ((he->target()->point() != (*v)->point())
				|| (he->face() != main_face)) {
			he++;
			if (he == poly.halfedges_end()) {
				std::cout << "Cannot find halfedge corresponding to vertex."
						<< std::endl;
				return false;
			}
		}
		fh = tev.compute_visibility(query_point, he, visibility_arr);
	} else if ((e = boost::get<VisibilityArrangement::Halfedge_const_handle>(
			&pl_result))) {
		// Located on halfedge. Find halfedge that has polygon interior as face.
		VisibilityArrangement::Halfedge_const_handle he =
				(*e)->face() == main_face ? (*e) : (*e)->twin();
		fh = tev.compute_visibility(query_point, he, visibility_arr);
	} else {
		std::cout << "Cannot locate query point on arrangement." << std::endl;
		return false;
	}
	// Result assertion.
	if (fh->is_fictitious()) {
		std::cout << "Visibility polygon is fictitious." << std::endl;
		return false;
	}
	if (fh->is_unbounded()) {
		std::cout << "Visibility polygon is unbounded." << std::endl;
		return false;
	}
	// Convert to polygon.
	VisibilityArrangement::Ccb_halfedge_circulator curr = fh->outer_ccb();
	*visibility_polygon = Polygon_2();
	do {
		visibility_polygon->push_back(curr->source()->point());
	} while (++curr != fh->outer_ccb());
	simplifyPolygon(visibility_polygon);
	if (visibility_polygon->is_clockwise_oriented())
		visibility_polygon->reverse_orientation();
	return true;
}


}// namespace polygon_coverage_planning
