#ifndef INTEGRATED_NAVIGATION_VELOCITY_DATA_HPP
#define INTEGRATED_NAVIGATION_VELOCITY_DATA_HPP

#include <deque>
#include <Eigen/Dense>

namespace integrated_navigation {
    class VelocityData {
    public:
        Eigen::Vector3d velocity;
        double time;

    public:
        static bool SyncData(const double sync_time, std::deque<VelocityData>& UnsyncedData, Eigen::Vector3d *G_v_I);
    };
}
#endif //INTEGRATED_NAVIGATION_VELOCITY_DATA_HPP