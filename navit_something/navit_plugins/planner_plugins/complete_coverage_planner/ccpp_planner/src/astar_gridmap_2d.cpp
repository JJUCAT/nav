#include "../include/astar_gridmap_2d.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <cinttypes>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>

using namespace std::placeholders;

namespace AStar {

PathFinder::PathFinder()
        : _open_set( CompareScore() ) {
    _allow_use_costmap = false;
    _allow_5x5_search = false;
    _obstacle_threshold = 0;
    setHeuristic(&Heuristic::manhattan);
    _directions = {{
        { -1, -1 },  { 0, -1 }, { 1, -1 }, //0 - 2
        { -1,  0 },             { 1,  0 }, //3 - 4
        { -1,  1 },  { 0, 1 },  { 1,  1 }, //5 - 7

        { -2, -2 }, { -1, -2 }, { 0, -2 }, { 1, -2 }, { 2, -2 }, //8 - 12
        { -2, -1 },                                   { 2, -1 }, //13 - 15
        { -2,  0 },                                   { 2,  0 }, //16 - 17
        { -2,  1 },                                   { 2,  1 }, //18 - 19
        { -2,  2 }, { -1,  2 }, { 0,  2 }, { 1,  2 }, { 2,  2 }  //20 - 23
    }};

    _direction_cost = {{
        14, 10, 14,
        10,     10,
        14, 10, 14,

        27, 22, 19, 22, 27,
        22,             22,
        19,             19,
        22,             22,
        27, 22, 19, 22, 27
    }};
}

PathFinder::~PathFinder() {}

bool PathFinder::setWorldData(const cv::Mat& map_img,
                              bool use_costmap,
                              double inflation_distance,
                              double map_resolution,
                              double cost_scaling_factor,
                              uint8_t color_threshold) {
    _obstacle_threshold = color_threshold;
    if (map_img.cols >= std::numeric_limits<int16_t>::max() ||
        map_img.rows >= std::numeric_limits<int16_t>::max()) {
        std::cout << "Either width or height exceed the maximum size allowed (32768) " << std::endl;
        return false;
    }

    _world_width  = map_img.cols;
    _world_height = map_img.rows;
    _gridmap.resize(map_img.cols * map_img.rows);

    // 二值化
    cv::threshold(map_img,_plan_map,200,255,cv::THRESH_BINARY);

    // 膨胀
    if(inflation_distance > 0.01) {
        int expend_distance_in_pixel = int(inflation_distance / map_resolution);

		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
			cv::Size(expend_distance_in_pixel, expend_distance_in_pixel));
		//cv::erode(_plan_map, _plan_map, element);                       //腐蚀
        cv::erode(_plan_map, _plan_map, cv::Mat(), cv::Point(-1, -1), expend_distance_in_pixel);
    }

    // get cost map
    if (use_costmap) {
        _allow_use_costmap = use_costmap;
        _cost_map = getPotentialFieldMap(_plan_map, cost_scaling_factor);
    }

    // set map data
    for (int y = 0; y < _plan_map.rows; y++) {
        for (int x = 0; x < _plan_map.cols; x++) {
            _gridmap[_plan_map.cols*y + x].world = _plan_map.at<unsigned char>(y, x);
        }
    }

    return true;
}

void PathFinder::setHeuristic(HeuristicFunction heuristic_) {
    _heuristic = std::bind(heuristic_, _1, _2);
}

cv::Mat PathFinder::getPotentialFieldMap(const cv::Mat& map,
                                         double ceofficient) {
    /// get dist tf map
    cv::Mat edt_obs_w;
    cv::distanceTransform(map, edt_obs_w, cv::DIST_L2, 3);

    // ceofficient map
    cv::Mat ceofficient_obs_w = ceofficient * edt_obs_w;

    // normal max dist
    for (int i=0; i<ceofficient_obs_w.rows; i++) {
        for (int j=0; j<ceofficient_obs_w.cols; j++) {
            if (ceofficient_obs_w.at<float>(i, j) > 255.0) {
                ceofficient_obs_w.at<float>(i, j) = 255.0;
            }
        }
    }

    // invert dist
    cv::Mat dist_tf_obs_w = 255 - ceofficient_obs_w;

    /// get m
    float max_val = 0;
    for (int i=0; i<dist_tf_obs_w.rows; i++) {
        for (int j=0; j<dist_tf_obs_w.cols; j++) {
            if (dist_tf_obs_w.at<float>(i, j) > max_val) {
                max_val = dist_tf_obs_w.at<float>(i, j);
            }
        }
    }
    float m = std::max(max_val, float(1.0));

    /// normal float to unsigned char
    cv::Mat out_obs_w(dist_tf_obs_w.size(), CV_8UC1);
    for (int i=0; i<out_obs_w.rows; i++) {
        for (int j=0; j<out_obs_w.cols; j++) {
            out_obs_w.at<unsigned char>(i, j) = (unsigned char)(dist_tf_obs_w.at<float>(i, j) * 255.0 / m);
        }
    }

    return out_obs_w;
}

void PathFinder::clean() {
    while( !_open_set.empty() ) {
        _open_set.pop();
    }

    for(Cell& cell: _gridmap) {
        cell.cost_G = std::numeric_limits<decltype(cell.cost_G)>::max();
        cell.already_visited = false;
    }
}


int PathFinder::toIndex(Coord2D pos)
{
	return (this->_world_width*pos.y + pos.x);
}

CoordinateList PathFinder::findPath(Coord2D startPos, Coord2D goalPos) {
    clean();

    const int startIndex = toIndex(startPos);
	const ScoreCoordPair a = { 0, startPos };
    _open_set.push(a);
    _gridmap[startIndex].cost_G = 0.0;

    bool solution_found = false;

    while (! _open_set.empty() ) {
        Coord2D currentCoord = _open_set.top().second;
        _open_set.pop();

        if (currentCoord == goalPos) {
            solution_found = true;
            break;
        }

        int currentIndex = toIndex(currentCoord);
        Cell& currentCell = _gridmap[ currentIndex ];
        currentCell.already_visited = true;

        std::array<bool,24> mask;
        std::array<Coord2D,24> newDirections;
        mask.fill(true);

        const int max_index = (_allow_5x5_search ? 24:8);

        for (int i=0; i<max_index ; i++) {
            newDirections[i] = currentCoord + _directions[i];
        }

        for (int i=0; i<8; i++) {
            mask[i] =  detectCollision( newDirections[i] );
        }
        if( _allow_5x5_search ) {
            mask[9]  = mask[0] || mask[1] || detectCollision( newDirections[9] );
            mask[11] = mask[1] || mask[2] || detectCollision( newDirections[11] );

            mask[13] = mask[0] || mask[3] || detectCollision( newDirections[13] );
            mask[14] = mask[2] || mask[4] || detectCollision( newDirections[14] );

            mask[17] = mask[3] || mask[5] || detectCollision( newDirections[17] );
            mask[18] = mask[4] || mask[7] || detectCollision( newDirections[18] );

            mask[20] = mask[5] || mask[6] || detectCollision( newDirections[20] );
            mask[22] = mask[6] || mask[7] || detectCollision( newDirections[22] );
        }

        for (int i = 0; i < max_index; ++i) {
            if (mask[i]) {
                continue;
            }

            Coord2D newCoordinates(newDirections[i]);
            size_t newIndex = toIndex(newCoordinates);
            Cell& newCell = _gridmap[newIndex];

            if (newCell.already_visited) {
                continue;
            }

            auto new_cost = currentCell.cost_G + _direction_cost[i];
            if (_allow_use_costmap) {
                //new_cost += float(_cost_map.at<uchar>(newCoordinates.y, newCoordinates.x) / 16.0);
                new_cost += float(_cost_map.at<uchar>(newCoordinates.y, newCoordinates.x));
            }

            if (new_cost < newCell.cost_G) {
                auto H = _heuristic( newCoordinates, goalPos );
                _open_set.push( { new_cost + H, newCoordinates } );
                newCell.cost_G = new_cost;
                newCell.path_parent = currentCoord;
            }
        }
    }

    CoordinateList path;
    if ( solution_found ) {
        Coord2D coord = goalPos;
        while (coord != startPos) {
            path.push_back( coord );
            coord = cell(coord).path_parent;
        }
    }
    else {
        std::cout << "Solution not found." << std::endl;
        std::cout << " open set size= " << _open_set.size() << std::endl;
    }

    // 修正轨迹方向
    std::reverse(path.begin(),path.end());

    return path;
}

void PathFinder::showPath(const CoordinateList& path,
                          bool show_visited_cell) {
    cv::Mat display_img;
    if (_allow_use_costmap) {
        display_img = 255 - _cost_map;
    } else {
        _plan_map.copyTo(display_img);
    }

    if (show_visited_cell) {
        for (uint32_t y=0; y<_world_height; y++) {
            for (uint32_t x=0; x<_world_width; x++) {
                if ( _gridmap[ y*_world_width + x ].already_visited ) {
                    cv::circle(display_img,cv::Point(x, y),1,cv::Scalar(160),-1);
                }
            }
        }
    }

    if (!path.empty()) {
        for (int i=0; i<(path.size()-1); i++) {
            cv::Point ss,ee;
            ss.x = path[i].x;
            ss.y = path[i].y;
            ee.x = path[i+1].x;
            ee.y = path[i+1].y;
            cv::circle(display_img,ss,2,cv::Scalar(64),-1);
        }
    }
}

bool PathFinder::detectCollision(Coord2D coordinates) {
    return (coordinates.x < 0 || coordinates.x >= _world_width ||
            coordinates.y < 0 || coordinates.y >= _world_height ||
            cell(coordinates).world <= _obstacle_threshold );
}

uint32_t Heuristic::manhattan(Coord2D source, Coord2D target) {
    auto delta = Coord2D( (source.x - target.x), (source.y - target.y) );
    return static_cast<uint32_t>(10 * ( abs(delta.x) + abs(delta.y)));
}

uint32_t Heuristic::euclidean(Coord2D source, Coord2D target) {
    auto delta = Coord2D( (source.x - target.x), (source.y - target.y) );
    return static_cast<uint32_t>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

uint32_t Heuristic::octagonal(Coord2D source, Coord2D target) {
    auto delta = Coord2D( abs(source.x - target.x), abs(source.y - target.y) );
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

} // namespace AStar
