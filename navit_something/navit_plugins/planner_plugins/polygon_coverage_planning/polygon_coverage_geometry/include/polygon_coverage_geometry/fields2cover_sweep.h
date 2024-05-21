#ifndef FIELDS_TO_COVER_SWEEP_H_
#define FIELDS_TO_COVER_SWEEP_H_

#include <fields2cover.h>

#include "polygon_coverage_geometry/cgal_definitions.h"
#include "polygon_coverage_geometry/visibility_graph.h"

namespace polygon_coverage_planning {

bool fields2CoverComputeSweep(const int turn_type, const int route_type,
                              const double turn_radius,
                              const double robot_width, const double op_width,
                              const Polygon_2& in, const Direction_2& dir, const double shrink_factor,
                              bool counter_clockwise,
                              std::vector<Point_2>* waypoints);

bool fields2CoverComputeAllSweeps(
    const int turn_type, const int route_type, const double turn_radius,
    const double robot_width, const double max_sweep_offset, double shrink_factor,
    const Polygon_2& poly, std::vector<std::vector<Point_2>>* cluster_sweeps);

}  // namespace polygon_coverage_planning

#endif  // FIELDS_TO_COVER_NODE_H_
