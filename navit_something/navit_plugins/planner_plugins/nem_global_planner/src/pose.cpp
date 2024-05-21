#include "nem_global_planner/pose.h"
#include <limits>

float Pose::distance(const Pose &pose) const {
    float diffX = pose.x - x;
    float diffY = pose.y - y;

    return sqrt(diffX * diffX + diffY * diffY);
}

Pose Pose::createInvalid() {
    return Pose(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                -100);
}

bool Pose::isValid() const { return x == std::numeric_limits<float>::max() && y == std::numeric_limits<float>::max(); }
