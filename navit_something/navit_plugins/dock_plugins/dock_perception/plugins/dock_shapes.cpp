#ifndef DOCK_SHAPE_V_H
#define DOCK_SHAPE_V_H

#include <dock_perception/dock_shape_base.h>

namespace dock_perception {
    
    class DockShapeV : public DockShapeBase
    {
        public:
            Cloud getIdealCloud(ros::NodeHandle &pnh) override{
                Cloud ideal_cloud;
                // V形状, theta是夹角，l是长度
                // y = -0.5 * tan(theta/2) * x , y in [0,l*sin(theta/2)], x in [0, l*cos(theta/2)]
                // y = 0.5 * tan(theta/2) * x, y in [-l*sin(theta/2), 0], x in [0, l*cos(theta/2)]
                double l = 0.25; //m
                double theta_deg = 120.0; //deg
                pnh.param("v_shape/l", l, l);
                pnh.param("v_shape/theta_deg", theta_deg, theta_deg);
                double theta = theta_deg * M_PI/180.0;
                for (double x = 0.0; x < l * std::cos(theta/2.0); x+= 0.002)
                {
                    geometry_msgs::Point p;
                    p.x = x;
                    p.y = - std::tan(theta/2.0) * x + l * std::sin(theta/2.0);
                    p.z = 0.0;
                    ideal_cloud.push_back(p);
                    p.y = std::tan(theta/2.0) * x - l * std::sin(theta/2.0);
                    ideal_cloud.insert(ideal_cloud.begin(), p);
                }

                ROS_INFO("V shaped dock cloud is loaded");

                return ideal_cloud; 
            }
    };
    
    class DockShapeMir : public DockShapeBase
    {
        public:
            Cloud getIdealCloud(ros::NodeHandle &pnh) override{
                Cloud ideal_cloud;
                // V形状, theta是夹角，l是长度
                // y = -0.5 * tan(theta/2) * x , y in [0,l*sin(theta/2)], x in [0, l*cos(theta/2)]
                // y = 0.5 * tan(theta/2) * x, y in [-l*sin(theta/2), 0], x in [0, l*cos(theta/2)]
                double l = 0.25; //m
                double theta_deg = 120.0; //deg
                double bottom_l = 0.3;
                pnh.param("mir_shape/v_shape_l", l, l);
                pnh.param("mir_shape/v_shape_theta_deg", theta_deg, theta_deg);
                pnh.param("mir_shape/bottom_l",bottom_l, bottom_l);
                double theta = theta_deg * M_PI/180.0;
                for (double x = 0.0; x < l * std::cos(theta/2.0); x+= 0.005)
                {
                    geometry_msgs::Point p;
                    p.x = x;
                    p.y = - std::tan(theta/2.0) * x + l * std::sin(theta/2.0);
                    p.z = 0.0;
                    ideal_cloud.push_back(p);
                    p.y = std::tan(theta/2.0) * x - l * std::sin(theta/2.0);
                    ideal_cloud.insert(ideal_cloud.begin(), p);

                    if (x == 0)
                    {
                        for(int i = 0; i < bottom_l/0.005; i++)
                        {
                            p.y = std::tan(theta/2.0) * x - l * std::sin(theta/2.0) - i * 0.005;
                            ideal_cloud.insert(ideal_cloud.begin(), p);
                        }
                    }
                }

                ROS_INFO("Mir shaped dock cloud is loaded");

                return ideal_cloud; 
            }
    };

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dock_perception::DockShapeV, dock_perception::DockShapeBase)
PLUGINLIB_EXPORT_CLASS(dock_perception::DockShapeMir, dock_perception::DockShapeBase)
#endif
