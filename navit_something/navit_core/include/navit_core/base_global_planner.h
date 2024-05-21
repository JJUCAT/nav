#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

namespace navit_core {

    class GlobalPlanner
    {
        public:
            using Ptr = boost::shared_ptr<GlobalPlanner>;

            virtual ~GlobalPlanner(){}

            virtual void initialize(const std::string& name,
                                    const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) = 0;

            virtual nav_msgs::Path makePlan(const geometry_msgs::PoseStamped& start,
                                            const geometry_msgs::PoseStamped& goal) = 0;

            // 传统点边模式需要
            virtual bool initNemData(const std::string &node_file_path, const std::string &edge_file_path)
            {
                return false;
            }
    };

    class CoveragePlanner
    {
        public:
            using Ptr = boost::shared_ptr<CoveragePlanner>;

            virtual ~CoveragePlanner(){}

            virtual void initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) = 0;

            virtual bool makePlan(const nav_msgs::Path& edge_path, const geometry_msgs::Pose& start_pose,
                                  nav_msgs::Path& coverage_path) = 0;

			/**@brief 全覆盖算法插件接口
			 * @param coverage_area 全覆盖区域
			 * @param holes 孔洞区域
			 * @param start 起点
			 * @param end 终点
			 * @param coverage_path 全覆盖路径
			 * @param contour_path 轮廓路径
			 * @return 是否成功
			 */
            virtual bool makePlan(const geometry_msgs::Polygon& coverage_area,
                                  const std::vector<geometry_msgs::Polygon>& holes,
                                  const geometry_msgs::Pose& start,
                                  const geometry_msgs::Pose& end,
                                  std::vector<nav_msgs::Path>& coverage_paths,
                                  std::vector<nav_msgs::Path>& contour_paths) {return false;};
    };

    class SmoothPathPlanner
    {
        public:
            using Ptr = boost::shared_ptr<SmoothPathPlanner>;

            virtual ~SmoothPathPlanner() {}

            virtual void initialize(const std::string& name) = 0;

            virtual bool makePlan(const nav_msgs::Path& rough_path, nav_msgs::Path& smooth_path) = 0;
    };
}


#endif
