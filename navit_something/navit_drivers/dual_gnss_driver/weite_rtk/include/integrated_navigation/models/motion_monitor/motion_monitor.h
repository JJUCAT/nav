#ifndef INTEGRATED_MOTION_MONITOR_H
#define INTEGRATED_MOTION_MONITOR_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"
#include <limits>

namespace integrated_navigation {
    class MotionMonitor{
    public:
        MotionMonitor(ros::NodeHandle &nh);
        ~MotionMonitor();

        bool CheckState(double ax, double ay, double az,
                        double wx, double wy, double wz) const;

    private:

        bool initialized_filter_;
        double kGravity_;
        double gain_acc_;
        double bias_alpha_;
        bool do_bias_estimation_;
        bool do_adaptive_gain_;
        double time_prev_;
        bool initialized_;
        bool steady_state_;

        // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
        // the orientation of the fixed frame wrt the body frame.
        double q0_, q1_, q2_, q3_;

        // Angular velocities;
        double wx_prev_, wy_prev_, wz_prev_;

        // Bias in angular velocities;
        double wx_bias_, wy_bias_, wz_bias_;

        double kAngularVelocityThreshold_;
        double kAccelerationThreshold_;
        double kDeltaAngularVelocityThreshold_;
    };
} //namespace integrated_navigation

#endif // INTEGRATED_MOTION_MONITOR_H