/*
 * TravellingSolver.h
 *
 *  Created on: 2023年5月25日
 *      Author: yjh
 */

#ifndef FCPP_ALGORITHM_INCLUDE_TRAVELLINGSOLVER_H_
#define FCPP_ALGORITHM_INCLUDE_TRAVELLINGSOLVER_H_

#include "fcpp_algorithm/common/def.h"

namespace fcpp {

class CellNode {
public:
	CellNode() {
		isVisited = false;
		isCleaned = false;
		parentIndex = INT_MAX;
		cellIndex = INT_MAX;
	}
	bool isVisited;
	bool isCleaned;
	int parentIndex;
	std::deque<int> neighbor_indices;
	int cellIndex;
};

class TravellingSolver {
public:
	TravellingSolver();
	virtual ~TravellingSolver();
	void SetDecompositions(const std::vector<Polygon_2> &decompositions);
	virtual void solve(int cell_index, int &unvisited_counter,
			std::deque<CellNode> &path) =0;
	std::deque<int> getTravellingPath(int first_cell_index);
	std::vector<CellNode> getCellGraph();
protected:
	void calculateDecompositionAdjacency(
			const std::vector<Polygon_2> &decompositions);
protected:
	std::vector<CellNode> cell_graph_;
};

class BFS_TSP: public TravellingSolver {
public:
	BFS_TSP();
	virtual ~BFS_TSP();
	// DFS
	void solve(int cell_index, int &unvisited_counter,
			std::deque<CellNode> &path) override;
};

}
#endif /* FCPP_ALGORITHM_INCLUDE_TRAVELLINGSOLVER_H_ */
