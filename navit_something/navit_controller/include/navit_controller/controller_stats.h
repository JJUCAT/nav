#ifndef NAVIT_CONTROLLER__CONTROLLER_STATS_H
#define NAVIT_CONTROLLER__CONTROLLER_STATS_H

#include <vector>
#include <chrono>
#include <string>
#include <numeric>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <navit_msgs/ControllerStats.h>

namespace navit_controller {

    struct Stats 
    {
        std::string controller_plugin;
        std::vector<double> cpu_time_ms;
        std::vector<double> vx;
        std::vector<double> vy;
        std::vector<double> omega;
        bool success = false;
        double average_cpu_time_ms;
        double current_cpu_time_ms;
        geometry_msgs::Twist average_cmd_vel;
        geometry_msgs::Twist current_cmd_vel;
    };

    class ControllerStats
    {
        public:
            ControllerStats()
            {
                stats_pub_ = nh_.advertise<navit_msgs::ControllerStats>("controller_stats",1); 
            }
            ~ControllerStats(){};

            void tic()
            {
                tic_ = std::chrono::high_resolution_clock::now();
            }

            void toc()
            {
                toc_ = std::chrono::high_resolution_clock::now();
                double cpu_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(toc_ - tic_).count();
                stats_.cpu_time_ms.push_back(cpu_time_ms);
                stats_.current_cpu_time_ms = cpu_time_ms;
                stats_.average_cpu_time_ms = std::accumulate(stats_.cpu_time_ms.begin(),
                                                             stats_.cpu_time_ms.end(),
                                                             0.0) / stats_.cpu_time_ms.size();
                //ROS_INFO("average cpu time %f ms", stats_.average_cpu_time_ms);
            }

            void setCmdVel(const geometry_msgs::Twist& current_cmd_vel)
            {
                geometry_msgs::Twist tmp_cmd_vel = current_cmd_vel;
                stats_.current_cmd_vel = tmp_cmd_vel;
                stats_.vx.push_back(tmp_cmd_vel.linear.x);
                stats_.vy.push_back(tmp_cmd_vel.linear.y);
                stats_.omega.push_back(tmp_cmd_vel.angular.z);

                double vx = std::accumulate(stats_.vx.begin(), stats_.vx.end(), 0.0) / stats_.vx.size();
                double vy = std::accumulate(stats_.vy.begin(), stats_.vy.end(), 0.0) / stats_.vx.size();
                double omega = std::accumulate(stats_.omega.begin(), stats_.omega.end(), 0.0) / stats_.vx.size();

                stats_.average_cmd_vel.linear.x = vx;
                stats_.average_cmd_vel.linear.y = vy;
                stats_.average_cmd_vel.angular.z = omega;
            }

            void printStats()
            {
                navit_msgs::ControllerStats stats_msg;
                stats_msg.header.stamp = ros::Time::now();
                stats_msg.controller_plugin = stats_.controller_plugin;
                stats_msg.cpu_time_ms = stats_.current_cpu_time_ms; 
                stats_msg.average_cpu_time_ms = stats_.average_cpu_time_ms;
                stats_msg.success = stats_.success;
                stats_msg.average_cmd_vel = stats_.average_cmd_vel;
                stats_msg.current_cmd_vel = stats_.current_cmd_vel; 

                stats_pub_.publish(stats_msg);
            }

            void setControllerName(std::string name)
            {
                stats_.controller_plugin = name;
            }

            void setSuccess()
            {
                stats_.success = true;
            }

        private:
            Stats stats_;
            std::chrono::high_resolution_clock::time_point tic_,toc_;
            ros::NodeHandle nh_;
            ros::Publisher stats_pub_;
    };
}

#endif
