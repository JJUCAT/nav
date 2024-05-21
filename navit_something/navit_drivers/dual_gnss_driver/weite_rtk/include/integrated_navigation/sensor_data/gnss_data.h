#ifndef SRC_GNSS_DATA_HPP
#define SRC_GNSS_DATA_HPP

#include <deque>
#include <Eigen/Dense>

namespace integrated_navigation {
    struct GnssPositionData {
        double timestamp;                     // timestamp in second.

        unsigned int gps_week_num;
        double gps_seconds;

        // bestpos.
        std::string pos_sol_status;
        std::string pos_type;                 // novatel position type.
        Eigen::Vector3d lla;                  // WGS84, Latitude in degree, longitude in degree, and altitude in meter.
        Eigen::Matrix3d position_covariance;  // Covariance in m^2.
        Eigen::Vector3d position_sigma;

        // bestxyz.
        std::string xyz_sol_status;
        std::string xyz_type;
        Eigen::Vector3d velocity;             // novatel velocity in ECEF.
        double v_latency;
        Eigen::Matrix3d velocity_covariance;

        // heading.
        std::string heading_type;             // novatel heading type.
        double heading;                       // novatel heading, degree.
        double heading_covariance;            // novatel heading covariance.
        double heading_sigma;                 // novatel heading sigma.

        // rover status.
        unsigned int num_satellites1;         // novatel position satellite numbers.
        unsigned int num_satellites2;         // novatel heading satellite numbers.
        double age;
    };
    using GnssPositionDataPtr = std::shared_ptr<GnssPositionData>;
}
#endif //SRC_GNSS_DATA_HPP