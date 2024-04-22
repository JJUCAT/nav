#ifndef PLANNER_STATS_H
#define PLANNER_STATS_H

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <navit_planner/planner_server.h>
#include <navit_msgs/ComputePathAction.h>
#include <navit_msgs/PlannerStats.h>
#include <chrono>

namespace navit_planner{

using ActionGoal = navit_msgs::ComputePathGoalConstPtr;

    class PlannerStats {
        public:
            PlannerStats(){
                stats_pub_ = nh_.advertise<navit_msgs::PlannerStats>("planner_stats",1);
            }
            ~PlannerStats(){}

            void setStartTime()
            {
                start_ = std::chrono::high_resolution_clock::now();
            }

            void setEndTime()
            {
                end_ = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> t = end_ - start_;
                stats_.cpu_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t).count();
            }


            void setResult(const nav_msgs::Path& planned_path) 
            {
                planned_path_ = planned_path;
                if (planned_path_.poses.size() == 0)
                {
                    stats_.success = false;
                    return;
                }

                stats_.success = true;

                double length = 0.0;
                for(auto i = 0; i < planned_path_.poses.size() - 1; i++)
                {
                    double diff_x = planned_path_.poses[i+1].pose.position.x - planned_path_.poses[i].pose.position.x;
                    double diff_y = planned_path_.poses[i+1].pose.position.y - planned_path_.poses[i].pose.position.y;
                    length += std::hypot(diff_x, diff_y);
                }
                stats_.path_length_m = length;
            }

            void setActionGoal(const ActionGoal& goal)
            {
                goal_ = goal;
                stats_.planner_plugin = goal_->planner_plugin;
                stats_.start = goal->start;
                stats_.goal = goal->goal;
            }
            void printStats()
            {
                ROS_INFO_STREAM("Planner: "<< stats_.planner_plugin << "\n" <<
                                "Success: "<< ( stats_.success ? "true" : "false") << "\n" <<
                                "CPU time(ms): "<< stats_.cpu_time_ms<< "\n" <<
                                "Path length(m): "<< stats_.path_length_m<< "\n" 
                                );
                navit_msgs::PlannerStats stats;
                stats.header.stamp = ros::Time::now();
                stats.planner_plugin = stats_.planner_plugin;
                stats.success = stats_.success;
                stats.cpu_time_ms = stats_.cpu_time_ms;
                stats.path_length_m = stats_.path_length_m;

                stats_pub_.publish(stats);
            }

        private:
            struct stats {
                std::string planner_plugin="navfn";
                bool success=false;
                double cpu_time_ms=0.0;
                double path_length_m=0.0;
                geometry_msgs::PoseStamped start;
                geometry_msgs::PoseStamped goal;
                nav_msgs::Path planned_path;
            } stats_;

            ros::NodeHandle nh_;

            std::chrono::high_resolution_clock::time_point start_;
            std::chrono::high_resolution_clock::time_point end_;

            ActionGoal goal_;
            nav_msgs::Path planned_path_;
            ros::Publisher stats_pub_;
    
    };
}
#endif
