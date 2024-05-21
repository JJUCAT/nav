//
// Created by fan on 23-1-4.
//

#ifndef NAVIT_CORE_BASE_GRAPH_SEARCH_H
#define NAVIT_CORE_BASE_GRAPH_SEARCH_H

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>


namespace navit_core
{

class Graph
{
public:
  using Ptr = boost::shared_ptr<Graph>;

  struct Edge
  {
    int64_t id;
    int64_t from;
    int64_t to;
    double cost;
  };

  void add_edge(int64_t id, int64_t from, int64_t to, double cost)
  {
    edges[id] = { id, from, to, cost };
    p2p_cost[from][to] = cost;
  }

  std::map<int64_t, Edge> edges;
  std::map<int64_t, std::map<int64_t, double>> p2p_cost;
};

struct SearchResultPath
{
  double cost;                // 代价
  std::vector<uint64_t> path;  // 输出路径。 path = from + ... + to
};

struct SearchResultRoute
{
  double cost;                            // 代价
  std::vector<int64_t> via_points_order;  // 经过点顺序。via_points_order = from
                                          // + via_points_ + to
  std::vector<SearchResultPath> route;    // route = path1 + path2 + path...
};

class BaseGraphSearch
{
public:
  using Ptr = boost::shared_ptr<BaseGraphSearch>;

  virtual ~BaseGraphSearch() = default;

  virtual void initialize(const std::string& name, const Graph::Ptr& graph)
  {
    name_ = name;
    graph_ = graph;
  }

  /**
   * search 搜索一条起点到终点的路径
   * @param from 起点
   * @param to 终点
   * @param out_result
   * @return
   */
  virtual bool search(int64_t from, int64_t to, SearchResultPath& out_result) = 0;

  /**
   * search 搜索一条起点到终点的路径 按顺序经过way_points
   * @param way_points
   * @param out_result
   * @return
   */
  bool search(const std::vector<int64_t>& way_points, SearchResultRoute& out_result)
  {
    out_result.via_points_order = way_points;
    for (int i = 0; i < way_points.size() - 1; i++)
    {
      SearchResultPath path;
      if (!search(way_points[i], way_points[i + 1], path))
      {
        return false;
      }
      out_result.cost += path.cost;
      out_result.route.push_back(path);
    }
    return true;
  }

protected:
  std::string name_;
  Graph::Ptr graph_;
};

class BaseTspSearch
{
public:
  using Ptr = boost::shared_ptr<BaseTspSearch>;

  virtual ~BaseTspSearch() = default;

  virtual void initialize(const std::string& name, const Graph::Ptr& graph)
  {
    name_ = name;
    graph_ = graph;
  }

  /**
   * search  搜索一条起点到终点的路径 需经过所有way_points，并保证路线最短
   * @param way_points
   * @param out_result
   * @return
   */
  virtual bool search(const std::vector<int64_t>& way_points, SearchResultRoute& out_result) = 0;

  virtual bool search(const int64_t& start, 
                      const int64_t& goal, 
                      const std::vector<int64_t>& way_points,
                      const Graph &graph,
                      std::vector<int64_t>& out_result) {return false;}

private:
  std::string name_;
  Graph::Ptr graph_;
};

}  // namespace navit_core

#endif  // NAVIT_CORE_BASE_GRAPH_SEARCH_H
