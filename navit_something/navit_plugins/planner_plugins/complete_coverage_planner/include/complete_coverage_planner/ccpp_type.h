#ifndef CCPP_TYPE_H
#define CCPP_TYPE_H

namespace ccpp_planner 
{
    // special for ccpp
    enum CoverageDirection
    {
        HORIZONTAL,
        VERTICAL
    };

    enum CoverageStartDirection
    {
        LEFTUP,
        LEFTDOWN,
        RIGHTUP,
        RIGHTDOWN
    };

    enum MapDirection
    {
        UP,
        DOWN,
        LEFT,
        RIGHT
    };

    struct MapPose
    {
        MapPose(int x = 0, int y = 0, MapDirection direction = MapDirection::RIGHT)
        {
            this->x = x;
            this->y = y;
            this->direction = direction;
        }

        int x = 0; // [pixel]
        int y = 0; // [pixel]
        MapDirection direction = MapDirection::RIGHT;
    };

    struct RobotPose
    {
        MapPose current;    // robot current pose on map

        MapPose front;      // robot front position pose on map
        MapPose back;       // robot back position pose on map
        MapPose left;       // robot left position pose on map
        MapPose right;      // robot right position pose on map
    };
}

#endif //CCPP_TYPE_H
