#ifndef _ASTAR_GRIDMAP_2D_
#define _ASTAR_GRIDMAP_2D_

#include <array>
#include <vector>
#include <functional>
#include <set>
#include <map>
#include <unordered_map>
#include <queue>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace AStar 
{

typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;

enum 
{
    OBSTACLE = 0,
    EMPTY    = 255
};

struct Coord2D 
{
    int16_t x, y;

    Coord2D(): x(-1), y(-1) {}
    Coord2D(int x_, int y_): x(x_), y(y_) {}

    bool operator == (const Coord2D& other) const {
        return (x == other.x && y == other.y);
    }

    bool operator != (const Coord2D& other) const {
        return !(*this == other);
    }

    Coord2D operator + (const Coord2D& other) const {
        return { x + other.x, y + other.y };
    }
};

typedef std::function<uint32_t(Coord2D, Coord2D)> HeuristicFunction;
typedef std::vector<Coord2D> CoordinateList;
typedef std::pair<uint32_t, Coord2D> ScoreCoordPair;

struct Cell {
    uint8_t  world;
    bool     already_visited;
    Coord2D  path_parent;
    float    cost_G;
};

struct CompareScore {
    //Note: we want the priority_queue to be ordered from smaller to larger
    bool operator() (const ScoreCoordPair& a,
                     const ScoreCoordPair& b) {
        return a.first > b.first;
    }
};

class PathFinder {
public:

    PathFinder();
    ~PathFinder();

    bool setWorldData(const cv::Mat& map_img,
                      bool use_costmap = false,
                      double inflation_distance = 0,
                      double map_resolution = 0.05,
                      double cost_scaling_factor = 10,
                      uint8_t color_threshold = 20);

    void setHeuristic(HeuristicFunction heuristic_);

    void allow5by5(bool allow = false) {
        _allow_5x5_search = allow;
    }

    /// Function that performs the actual A* computation.
    CoordinateList findPath(Coord2D source_, Coord2D target_);

    /// Show the resulting solution use OpenCV. Useful for debugging.
    void showPath(const CoordinateList& path,
                  bool show_visited_cell = false);

private:

    cv::Mat getPotentialFieldMap(const cv::Mat& map,
                                 double ceofficient = 16);

    Cell& cell(Coord2D coordinates_) {
        return _gridmap[coordinates_.y*_world_width + coordinates_.x];
    }
    bool detectCollision(Coord2D coordinates);
    void clean();
	int toIndex(Coord2D pos);

    bool _allow_use_costmap;

	

    cv::Mat _cost_map;

    cv::Mat _plan_map;

    HeuristicFunction _heuristic;
    uint32_t _world_width;
    uint32_t _world_height;
    bool _allow_5x5_search;

    std::array<Coord2D,24>  _directions;
    std::array<uint32_t,24> _direction_cost;
    std::priority_queue<ScoreCoordPair, std::vector<ScoreCoordPair>, CompareScore> _open_set;

    std::vector<Cell> _gridmap;

    uint8_t _obstacle_threshold;
};

class Heuristic {
public:
    static uint32_t manhattan(Coord2D source_, Coord2D target_);
    static uint32_t euclidean(Coord2D source_, Coord2D target_);
    static uint32_t octagonal(Coord2D source_, Coord2D target_);
};

} // namespace AStar

#endif // _ASTAR_GRIDMAP_2D_
