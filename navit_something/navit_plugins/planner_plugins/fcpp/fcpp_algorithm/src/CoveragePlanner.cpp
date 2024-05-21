/*
 * CoveragePlanner.cpp
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#include "fcpp_algorithm/CoveragePlanner.h"

namespace polygon_coverage_planning {

CoveragePlanner::CoveragePlanner() {
}

CoveragePlanner::~CoveragePlanner() {
	decompose_.reset();
	tsp_solver_.reset();
	sweep_.reset();
	polygon_.reset();
}

bool CoveragePlanner::init(const CoveragePlannerConfig &config) {
	std::unique_lock<std::mutex>(config_mutex);
	config_ = config;
	if (config_.decompose_type == 0) {
		decompose_.reset(new fcpp::BCD());
	} else if (config_.decompose_type == 1) {
		decompose_.reset(new fcpp::TCD());
	}
	// todo use the default solver
	tsp_solver_.reset(new fcpp::BFS_TSP());
	sweep_.reset(new fcpp::SimpleSweepStrategy());
	return true;
}

bool CoveragePlanner::Update(const std::vector<Point> &outer_poly,
		const std::vector<std::vector<Point>> &inner_poly) {
	Polygon_2 outer_polygon;
	for (const auto &point : outer_poly) {
		outer_polygon.push_back(Point_2(point.x_, point.y_));
	}
	std::vector<Polygon_2> holes(inner_poly.size());
	for (unsigned int i = 0; i < inner_poly.size(); i++) {
		for (const auto &point : inner_poly[i]) {
			holes[i].push_back(Point_2(point.x_, point.y_));
		}
	}
	polygon_.reset(
			new PolygonWithHoles(outer_polygon, holes.begin(), holes.end()));
	return true;
}

bool CoveragePlanner::SolveSingleBow(const Point start) {
	// first : update cell decomposition
	way_points.clear();
	decompose_->setPolygonWithHoles(*polygon_);
	std::vector<Polygon_2> bcd_cells;
	if (!decompose_->compute(bcd_cells) || bcd_cells.size() == 0) {
		//todo add log info
		return false;
	}
	// second: build the adjacency graph and build the travel route
	int starting_cell_idx = decompose_->getCellIndexOfPoint(bcd_cells,
			Point_2(start.x_, start.y_));
	tsp_solver_->SetDecompositions(bcd_cells);
	DLOG(INFO) << " starting_cell_idx: " << starting_cell_idx;
	auto cell_idx_path = tsp_solver_->getTravellingPath(starting_cell_idx);
	//  third: build the sweep strategy in every cell
	std::vector<std::vector<Point_2>> cells_sweeps;
	for (size_t i = 0; i < bcd_cells.size(); ++i) {
		std::vector<Point_2> cell_sweep; // Compute all cluster sweeps.
		Direction_2 best_dir;
		decompose_->findBestSweepDir(bcd_cells[i], &best_dir);
		fcpp::VisibilityGraph vis_graph(bcd_cells[i]);
		sweep_->computeSweep(bcd_cells[i], vis_graph, config_.sweep_step,
				best_dir, config_.counter_clockwise, &cell_sweep);
		cells_sweeps.emplace_back(cell_sweep);
	}
	way_points.emplace_back(WaypointReult(Point_2(start.x_, start.y_),true)); // re-connnect the sweep line
	auto cell_graph = tsp_solver_->getCellGraph();
	for (auto &idx : cell_idx_path) {
		if (!cell_graph[idx].isCleaned) {
			if (sweep_->doReverseNextSweep(way_points.back().point,
					cells_sweeps[idx])) {
				for (auto it = cells_sweeps[idx].rbegin();
						it != cells_sweeps[idx].rend(); ++it) {
					WaypointReult via_point;
					via_point.point = *it;
					if (it == cells_sweeps[idx].rbegin()
							|| it == (cells_sweeps[idx].rend()--)) {
						via_point.is_transfer = true;
					} else {
						via_point.is_transfer = false;
					}
					way_points.insert(way_points.end(), via_point);
				}
			} else {
				for (auto it = cells_sweeps[idx].begin();
						it != cells_sweeps[idx].end(); ++it) {
					WaypointReult via_point;
					via_point.point = *it;
					if (it == cells_sweeps[idx].begin()
							|| it == (cells_sweeps[idx].end()--)) {
						via_point.is_transfer = true;
					} else {
						via_point.is_transfer = false;
					}
					way_points.insert(way_points.end(), via_point);
				}
			}
			cell_graph[idx].isCleaned = true;
		}
	}
	return true;
}

bool CoveragePlanner::SolveDoubleBow(const Point start) {
	// first : update cell decomposition
	way_points.clear();
	decompose_->setPolygonWithHoles(*polygon_);
	std::vector<Polygon_2> bcd_cells, bcd_cells_double;
	if (!decompose_->compute(bcd_cells) || bcd_cells.size() == 0) {
		//todo add log info
		return false;
	}
	// second: build the adjacency graph and build the travel route
	int starting_cell_idx = decompose_->getCellIndexOfPoint(bcd_cells,
			Point_2(start.x_, start.y_));
	tsp_solver_->SetDecompositions(bcd_cells);
	DLOG(INFO) << " starting_cell_idx: " << starting_cell_idx;
	auto cell_idx_path = tsp_solver_->getTravellingPath(starting_cell_idx);
	//  third: build the sweep strategy in every cell
	std::vector<std::vector<Point_2>> cells_sweeps, cells_sweeps_double;
	// Single Bow
	for (size_t i = 0; i < bcd_cells.size(); ++i) {
		std::vector<Point_2> cell_sweep; // Compute all cluster sweeps.
		Direction_2 best_dir;
		decompose_->findBestSweepDir(bcd_cells[i], &best_dir);
		fcpp::VisibilityGraph vis_graph(bcd_cells[i]);
		sweep_->computeSweep(bcd_cells[i], vis_graph, config_.sweep_step,
				best_dir, config_.counter_clockwise, &cell_sweep);
		cells_sweeps.emplace_back(cell_sweep);
	}
	way_points.emplace_back(WaypointReult(Point_2(start.x_, start.y_),true)); // re-connnect the sweep line
	auto cell_graph = tsp_solver_->getCellGraph();
	for (auto &idx : cell_idx_path) {
		if (!cell_graph[idx].isCleaned) {
			if (sweep_->doReverseNextSweep(way_points.back().point,
					cells_sweeps[idx])) {
				for (auto it = cells_sweeps[idx].rbegin();
						it != cells_sweeps[idx].rend(); ++it) {
					WaypointReult via_point;
					via_point.point = *it;
					if (it == cells_sweeps[idx].rbegin()
							|| it == (cells_sweeps[idx].rend()--)) {
						via_point.is_transfer = true;
					} else {
						via_point.is_transfer = false;
					}
					way_points.insert(way_points.end(), via_point);
				}
			} else {
				for (auto it = cells_sweeps[idx].begin();
						it != cells_sweeps[idx].end(); ++it) {
					WaypointReult via_point;
					via_point.point = *it;
					if (it == cells_sweeps[idx].begin()
							|| it == (cells_sweeps[idx].end()--)) {
						via_point.is_transfer = true;
					} else {
						via_point.is_transfer = false;
					}
					way_points.insert(way_points.end(), via_point);
				}
			}
			cell_graph[idx].isCleaned = true;
		}
	}
	// four: Double Bow
	// A: re-compute the traversal ,the start point is the back
	starting_cell_idx = cell_idx_path.back();
	// B: update the bcd_cells and get the best bcd_direction and compute the bcd cells by direction
	Direction_2 bcd_dir;
	if (!decompose_->getBestDecomposeDir(bcd_dir)) {
		//todo add log info
		return false;
	}
	const CGAL::Aff_transformation_2<K> roatate(CGAL::ROTATION,
			CGAL::Direction_2<K>(0, 1), 1, 100);
	bcd_dir = bcd_dir.transform(roatate); // rotate 90 counter
	if (!decompose_->compute(bcd_cells_double, bcd_dir)
			|| bcd_cells_double.size() == 0) {
		//todo add log info
		return false;
	}
	tsp_solver_->SetDecompositions(bcd_cells_double);
	auto cell_idx_path_double = tsp_solver_->getTravellingPath(
			starting_cell_idx);
	for (size_t i = 0; i < bcd_cells_double.size(); ++i) {
		std::vector<Point_2> cell_sweep; // Compute all cluster sweeps.
		Direction_2 sweep_dir;
		decompose_->findBestSweepDir(bcd_cells_double[i], &sweep_dir);
		fcpp::VisibilityGraph vis_graph(bcd_cells_double[i]);
		sweep_->computeSweep(bcd_cells_double[i], vis_graph, config_.sweep_step,
				sweep_dir, config_.counter_clockwise, &cell_sweep);
		cells_sweeps_double.emplace_back(cell_sweep);
	}
	auto cell_graph_double = tsp_solver_->getCellGraph();
	printf("cell_idx_path_double size is %ld, cell_graph_double size is %ld \n", cell_idx_path_double.size(),cell_graph_double.size());
	for (auto &idx : cell_idx_path_double) {
		if (!cell_graph_double[idx].isCleaned) {
			if (sweep_->doReverseNextSweep(way_points.back().point,
					cells_sweeps_double[idx])) {
				for (auto it = cells_sweeps_double[idx].rbegin();
						it != cells_sweeps_double[idx].rend(); ++it) {
					WaypointReult via_point;
					via_point.point = *it;
					if (it == cells_sweeps_double[idx].rbegin()
							|| it == (cells_sweeps_double[idx].rend()--)) {
						via_point.is_transfer = true;
					} else {
						via_point.is_transfer = false;
					}
					way_points.insert(way_points.end(), via_point);
				}
			}
		 else {
			for (auto it = cells_sweeps_double[idx].begin();
					it != cells_sweeps_double[idx].end(); ++it) {
				WaypointReult via_point;
				via_point.point = *it;
				if (it == cells_sweeps_double[idx].begin()
						|| it == (cells_sweeps_double[idx].end()--)) {
					via_point.is_transfer = true;
				} else {
					via_point.is_transfer = false;
				}
				way_points.insert(way_points.end(), via_point);
			}
		}
		cell_graph_double[idx].isCleaned = true;
	}
	}
	return true;
}

// todo add the judge
void CoveragePlanner::getWayPoint(std::vector<Point> &waypoint) {
	for (auto &point : way_points) {
		if (point.is_transfer)
			waypoint.push_back(
					Point(CGAL::to_double(point.point.x()),
							CGAL::to_double(point.point.y()), true));
		else {
			waypoint.push_back(
					Point(CGAL::to_double(point.point.x()),
							CGAL::to_double(point.point.y()), false));
		}
	}
}

} //namespace polygon_coverage_planning
