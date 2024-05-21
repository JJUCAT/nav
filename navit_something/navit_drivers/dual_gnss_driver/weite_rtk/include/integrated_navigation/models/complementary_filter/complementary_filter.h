#ifndef INTEGRATED_NAVIGATION_COMPLEMENTARY_FILTER_H
#define INTEGRATED_NAVIGATION_COMPLEMENTARY_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class ComplementaryFilter{
    public:
        ComplementaryFilter(ros::NodeHandle &nh);
        ~ComplementaryFilter();

        void ImuDataProcess(const geometry_msgs::Vector3 &acc, const geometry_msgs::Vector3 &gyro, const double time);

        void getOrientation(double& q0, double& q1, double& q2, double& q3);

        bool checkState(double ax, double ay, double az,
                        double wx, double wy, double wz) const;

        void updateAngularVel(const double wx, const double wy, const double wz);

        bool updateBiases(double ax, double ay, double az,
                          double wx, double wy, double wz);

    private:
        void Update(const double acc_x, const double acc_y, const double acc_z,
                const double gyro_x, const double gyro_y, const double gyro_z, const double dt);

        void getMeasurement(
                double ax, double ay, double az,
                double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas);

        void normalizeVector(double& x, double& y, double& z);


        void getPrediction(
                double wx, double wy, double wz, double dt,
                double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred);

        void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3);

        void getAccCorrection(
                double ax, double ay, double az,
                double p0, double p1, double p2, double p3,
                double& dq0, double& dq1, double& dq2, double& dq3);

        void rotateVectorByQuaternion(double x, double y, double z,
                                      double q0, double q1, double q2, double q3,
                                      double& vx, double& vy, double& vz);

        double getAdaptiveGain(double alpha, double ax, double ay, double az);

        void scaleQuaternion(double gain,
                             double& dq0, double& dq1, double& dq2, double& dq3);

        void quaternionMultiplication(double p0, double p1, double p2, double p3,
                                      double q0, double q1, double q2, double q3,
                                      double& r0, double& r1, double& r2, double& r3);

        void invertQuaternion(
                double q0, double q1, double q2, double q3,
                double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv);

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

        // Bias in angular velocities;
        double wx_prev_, wy_prev_, wz_prev_;

        // Bias in angular velocities;
        double wx_bias_, wy_bias_, wz_bias_;

        double kAngularVelocityThreshold_;
        double kAccelerationThreshold_;
        double kDeltaAngularVelocityThreshold_;
    };
}
#endif //INTEGRATED_NAVIGATION_COMPLEMENTARY_FILTER_H