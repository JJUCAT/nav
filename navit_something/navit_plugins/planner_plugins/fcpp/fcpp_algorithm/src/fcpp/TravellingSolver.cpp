/*
 * TravellingSolver.cpp
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#include "fcpp_algorithm/fcpp/TravellingSolver.h"

namespace fcpp {

TravellingSolver::TravellingSolver() {

}

TravellingSolver::~TravellingSolver() {
	cell_graph_.clear();
}

void TravellingSolver::SetDecompositions(
		const std::vector<Polygon_2> &decompositions) {
	// clear and build the travel graph
	cell_graph_.clear();
	calculateDecompositionAdjacency(decompositions);
}

std::vector<CellNode> TravellingSolver::getCellGraph(){
	return cell_graph_;
}

void TravellingSolver::calculateDecompositionAdjacency(
		const std::vector<Polygon_2> &decompositions) {
	cell_graph_.resize(decompositions.size());
	for (size_t i = 0; i < decompositions.size() - 1; ++i) {
		cell_graph_[i].cellIndex = i;
		for (size_t j = i + 1; j < decompositions.size(); ++j) {
			PolygonWithHoles joined;
			if (CGAL::join(decompositions[i], decompositions[j], joined)) {
				cell_graph_[i].neighbor_indices.emplace_back(j);
				cell_graph_[j].neighbor_indices.emplace_back(i);
			}
		}
	}
	cell_graph_.back().cellIndex = decompositions.size() - 1;
}

std::deque<int> TravellingSolver::getTravellingPath(int first_cell_index) {
	std::deque<int> travelling_path;
	std::deque<CellNode> _cell_path;
	if (cell_graph_.size() == 1) {
		travelling_path.emplace_back(0);
	} else {
		int unvisited_counter = cell_graph_.size();
		solve(first_cell_index, unvisited_counter, _cell_path);
		std::reverse(_cell_path.begin(), _cell_path.end());
	}
	for (auto &cell : _cell_path) {
		travelling_path.emplace_back(cell.cellIndex);
	}
	return travelling_path;
}

BFS_TSP::BFS_TSP() :
		TravellingSolver() {
}

BFS_TSP::~BFS_TSP() {
}

void BFS_TSP::solve(int cell_index, int &unvisited_counter,
		std::deque<CellNode> &path) {
	if (!cell_graph_[cell_index].isVisited) {
		cell_graph_[cell_index].isVisited = true;
		unvisited_counter--;
	}
	path.emplace_front(cell_graph_[cell_index]);
	CellNode neighbor;
	int neighbor_idx = INT_MAX;
	for (unsigned int i = 0; i < cell_graph_[cell_index].neighbor_indices.size(); i++) {
		neighbor = cell_graph_[cell_graph_[cell_index].neighbor_indices[i]];
		neighbor_idx = cell_graph_[cell_index].neighbor_indices[i];
		if (!neighbor.isVisited) {
			break;
		}
	}
	if (!neighbor.isVisited) { // unvisited neighbor found
		cell_graph_[neighbor_idx].parentIndex =
				cell_graph_[cell_index].cellIndex;
		solve(neighbor_idx, unvisited_counter, path);
	}
	// unvisited neighbor not found
	else {
		// cannot go on back-tracking
		if (cell_graph_[cell_index].parentIndex == INT_MAX) {
			return;
		} else if (unvisited_counter == 0) {
			return;
		} else {
			solve(cell_graph_[cell_index].parentIndex, unvisited_counter, path);
		}
	}
}

}// namespcae fcpp
