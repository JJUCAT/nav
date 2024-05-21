#pragma imudata

#include <memory>
#include <vector>
#include <math.h>
#include <iostream>

#include "vanjee_driver/common/super_header.hpp"
#include <vanjee_driver/driver/decoder/wlr720/protocol/imu/complementary_filter.hpp>

#define HDL_Grabber_toRadians(x) ((x)*M_PI / 180.0)

namespace vanjee
{
  namespace lidar
  {
    class ImuParamGet720
    {
      public:
        double x_ang_k;
        double x_ang_b;
        double y_ang_k;
        double y_ang_b;
        double z_ang_k;
        double z_ang_b;

        double x_add_k;
        double x_add_b;
        double y_add_k;
        double y_add_b;
        double z_add_k;
        double z_add_b;

        double x_Zeropa;
        double y_Zeropa;
        double z_Zeropa;

        std::vector<double> imuXda;//emplace_back
        std::vector<double> imuYda;
        std::vector<double> imuZda;

        double rotate_B[2][2];

        int32_t meZeroPatime;
        int32_t aim_meZeroPatime;

        double dereTimepre;

        struct imuGetResultPa
        {
            double q0;
            double q1;
            double q2;
            double q3;

            double xang;
            double yang;
            double zang;

            double xadd;
            double yadd;
            double zadd;

            void init()
            {
                q0 = q1 = q2 = q3 = 0;
                xang = yang = zang = 0;
                xadd = yadd = zadd = 0;
            }
        };

        imuGetResultPa imuResultStu;

        //std::shared_ptr<imu_tools::ComplementaryFilter> m_fiter;
        imu_tools::ComplementaryFilter m_fiter;

        ImuParamGet720(int axisoffset)
        {
            x_ang_k = y_ang_k = z_ang_k = 1;
            x_ang_b = y_ang_b = z_ang_b = 0;

            x_add_k = y_add_k = z_add_k = 1;
            x_add_b = y_add_b = z_add_b = 0;

            x_Zeropa = y_Zeropa = z_Zeropa = 0;

            rotate_B[0][0] = cos(HDL_Grabber_toRadians(axisoffset));
            rotate_B[0][1] = -sin(HDL_Grabber_toRadians(axisoffset));
            rotate_B[1][0] = sin(HDL_Grabber_toRadians(axisoffset));
            rotate_B[1][1] = cos(HDL_Grabber_toRadians(axisoffset));

            meZeroPatime = 0;
            aim_meZeroPatime = 1000;

            dereTimepre = 0;

            imuResultStu.init();
        }

        ~ImuParamGet720()
        {

        }

        inline void setImuTempCalibrationParams(double x_k,double x_b,double y_k,double y_b,double z_k,double z_b)
        {
          x_ang_k = x_k;
          x_ang_b = x_b;
          y_ang_k = y_k;
          y_ang_b = y_b;
          z_ang_k = z_k;
          z_ang_b = z_b;
        }

        inline void setImuAcceCalibrationParams(double x_k,double x_b,double y_k,double y_b,double z_k,double z_b)
        {
          x_add_k = x_k;
          x_add_b = x_b;
          y_add_k = y_k;
          y_add_b = y_b;
          z_add_k = z_k;
          z_add_b = z_b;
        }

        inline void imuAngCorrbyTemp(double &x_ang, double &y_ang, double &z_ang , double imuTempreture)
        {
            x_ang -= x_ang_k * (imuTempreture-40) + x_ang_b;
            y_ang -= y_ang_k * (imuTempreture-40) + y_ang_b;
            z_ang -= z_ang_k * (imuTempreture-40) + z_ang_b;
        }

        inline void imuAddCorrbyKB(double &x_add, double &y_add, double &z_add)
        {
            x_add = x_add_k * x_add - x_add_b;
            y_add = y_add_k * y_add - y_add_b;
            z_add = z_add_k * z_add - z_add_b;
        }
        
        inline void rotateimu(double &x_ang, double &y_ang, double &z_ang , double &x_add, double &y_add, double &z_add, double imuTempreture)
        {
            double rotate_Ag[2] = {x_add, y_add};
            double rotate_Aa[2] = {x_ang, y_ang};

            x_add = rotate_Ag[0] * rotate_B[0][0] + rotate_Ag[1] * rotate_B[1][0];
            y_add = rotate_Ag[0] * rotate_B[0][1] + rotate_Ag[1] * rotate_B[1][1];

            x_ang = rotate_Aa[0] * rotate_B[0][0] + rotate_Aa[1] * rotate_B[1][0];
            y_ang = rotate_Aa[0] * rotate_B[0][1] + rotate_Aa[1] * rotate_B[1][1];
        }
        

        int32_t imuGetZeroPa(double x_ang, double y_ang, double z_ang)
        {
            x_Zeropa += x_ang / aim_meZeroPatime;
            y_Zeropa += y_ang / aim_meZeroPatime;
            z_Zeropa += z_ang / aim_meZeroPatime;

            return meZeroPatime++;
        }

        bool imuGet(double x_ang, double y_ang, double z_ang, double x_add, double y_add, double z_add , double temperature , double time)
        {
            imuAngCorrbyTemp(x_ang, y_ang, z_ang , temperature);
            
            x_add = (x_add / 1000 * 0.061 * 9.81);
            y_add = (y_add / 1000 * 0.061 * 9.81);
            z_add = (z_add / 1000 * 0.061 * 9.81);

            x_ang = x_ang / 1000 * 8.75 * 0.0174533;
            y_ang = y_ang / 1000 * 8.75 * 0.0174533;
            z_ang = z_ang / 1000 * 8.75 * 0.0174533;

            imuAddCorrbyKB(x_add, y_add, z_add);

            double l_dertatime;
            if (dereTimepre <= time)
            {
              l_dertatime = time - dereTimepre;
            }
            else
            {
              l_dertatime = 100000000 - dereTimepre + time;
            }

            l_dertatime = l_dertatime / 100000000;
            dereTimepre = time;

            if (meZeroPatime < aim_meZeroPatime)
            {
                imuGetZeroPa(x_ang, y_ang, z_ang);
                dereTimepre = time;
                return false;
            }
            else if (meZeroPatime == aim_meZeroPatime)
            {
                std::cout << "Begin publish Imu" << std::endl;
                meZeroPatime ++;
                dereTimepre = time;
            }

            x_ang = x_ang - x_Zeropa;
            y_ang = y_ang - y_Zeropa;
            z_ang = z_ang - z_Zeropa;

            imuResultStu.xang = x_ang;
            imuResultStu.yang = y_ang;
            imuResultStu.zang = z_ang;

            imuResultStu.xadd = x_add;
            imuResultStu.yadd = y_add;
            imuResultStu.zadd = z_add;

            static double q0 = 0;
            static double q1 = 0;
            static double q2 = 0;
            static double q3 = 0;

            m_fiter.update(x_add, y_add, z_add, x_ang, y_ang, z_ang, l_dertatime, q0, q1, q2, q3);
		        m_fiter.getOrientation(q0, q1, q2, q3);

            imuResultStu.q0 = q1;
            imuResultStu.q1 = q2;
            imuResultStu.q2 = q3;
            imuResultStu.q3 = q0;

            return true;
        }
    };

  }
}