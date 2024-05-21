#include "polygon_coverage_geometry/fields2cover_sweep.h"

#include <ros/assert.h>
#include <ros/console.h>

#include "polygon_coverage_geometry/weakly_monotone.h"

namespace polygon_coverage_planning {

bool fields2CoverComputeSweep(const int turn_type, const int route_type,
                              const double turn_radius,
                              const double robot_width, const double op_width,
                              const Polygon_2& in, const Direction_2& dir, const double shrink_factor,
                              bool counter_clockwise,
                              std::vector<Point_2>* waypoints) {
  F2CRobot robot{robot_width, op_width};
  robot.setMinRadius(turn_radius);
  F2CFields fields;
  F2COptim optim;

  waypoints->clear();

  auto positive = counter_clockwise ? 1 : -1;
  auto radian = atan2(positive * CGAL::to_double(dir.dx()),
                      positive * CGAL::to_double(dir.dy()));

  optim.best_angle = radian < 0 ? radian + 2 * M_PI : radian;

  ROS_WARN_STREAM("end ratate: "<<optim.best_angle);
  std::vector<F2CPoint> point_lists;
  for (auto v = in.vertices_begin(); v != in.vertices_end(); ++v) {
    point_lists.emplace_back(CGAL::to_double(v->x()), CGAL::to_double(v->y()));
  }
  point_lists.push_back(point_lists[0]);

  F2CLinearRing ring(point_lists);
  F2CCell no_headlands(ring);

  F2CSwaths swaths;
  f2c::sg::BruteForce swath_gen;

  swaths =
      swath_gen.generateSwaths(optim.best_angle, robot.op_width, no_headlands);
  ROS_WARN_STREAM("swaths size: "<<swaths.size());
  F2CSwaths route;
  switch (route_type) {
    case 0: {
      f2c::rp::BoustrophedonOrder swath_sorter;
      route = swath_sorter.genSortedSwaths(swaths);
      break;
    }
    case 1: {
      f2c::rp::SnakeOrder swath_sorter;
      route = swath_sorter.genSortedSwaths(swaths);
      break;
    }
    case 2: {
      f2c::rp::SpiralOrder swath_sorter(6);
      route = swath_sorter.genSortedSwaths(swaths);
      break;
    }
    case 3: {
      f2c::rp::SpiralOrder swath_sorter(4);
      route = swath_sorter.genSortedSwaths(swaths);
      break;
    }
  }

  F2CPath path;
  f2c::pp::PathPlanning path_planner;

  switch (turn_type) {
    case 0: {
      f2c::pp::DubinsCurves turn;
      path = path_planner.searchBestPath(robot, route, turn, shrink_factor);
      break;
    }
    case 1: {
      f2c::pp::DubinsCurvesCC turn;
      path = path_planner.searchBestPath(robot, route, turn, shrink_factor);
      break;
    }
    case 2: {
      f2c::pp::ReedsSheppCurves turn;
      path = path_planner.searchBestPath(robot, route, turn, shrink_factor);
      break;
    }
    case 3: {
      f2c::pp::ReedsSheppCurvesHC turn;
      path = path_planner.searchBestPath(robot, route, turn, shrink_factor);
      break;
    }
  }

  for (auto&& s : path.states) {
    waypoints->emplace_back(s.point.getX(), s.point.getY());
  }

  ROS_WARN_STREAM("fields2CoverComputeSweep waypoints size: "<<waypoints->size());
  return waypoints->size() > 0;
}

bool fields2CoverComputeAllSweeps(
    const int turn_type, const int route_type, const double turn_radius,
    const double robot_width, const double max_sweep_offset, double shrink_factor,
    const Polygon_2& poly, std::vector<std::vector<Point_2>>* cluster_sweeps) {

  ROS_ASSERT(cluster_sweeps);
  cluster_sweeps->clear();
  cluster_sweeps->reserve(2 * poly.size());

  // Find all sweepable directions.
  std::vector<Direction_2> dirs = getAllSweepableEdgeDirections(poly);

  // Compute all possible sweeps.
  visibility_graph::VisibilityGraph vis_graph(poly);
  for (const Direction_2& dir : dirs) {
    bool counter_clockwise = true;
    std::vector<Point_2> sweep;
    if (!fields2CoverComputeSweep(turn_type, route_type, turn_radius,
                                  robot_width, max_sweep_offset, poly, dir, shrink_factor,
                                  counter_clockwise, &sweep)) {
      ROS_ERROR_STREAM("Cannot compute counter-clockwise sweep.");
      continue;
    } else {
      ROS_ASSERT(!sweep.empty());
      cluster_sweeps->push_back(sweep);
      std::reverse(sweep.begin(), sweep.end());
      cluster_sweeps->push_back(sweep);
    }

    if (!fields2CoverComputeSweep(turn_type, route_type, turn_radius,
                                  robot_width, max_sweep_offset, poly, dir, shrink_factor,
                                  !counter_clockwise, &sweep)) {
      ROS_ERROR_STREAM("Cannot compute clockwise sweep.");
      continue;
    } else {
      ROS_ASSERT(!sweep.empty());
      cluster_sweeps->push_back(sweep);
      std::reverse(sweep.begin(), sweep.end());
      cluster_sweeps->push_back(sweep);
    }
  }
  return true;
}

}  // namespace polygon_coverage_planning