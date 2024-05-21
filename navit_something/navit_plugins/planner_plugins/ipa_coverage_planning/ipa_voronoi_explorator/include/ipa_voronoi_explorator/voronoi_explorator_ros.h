//
// Created by fan on 23-8-7.
//

#ifndef IPA_VORONOI_EXPLORATOR_BOUSTROPHEDON_EXPLORATOR_ROS_H
#define IPA_VORONOI_EXPLORATOR_BOUSTROPHEDON_EXPLORATOR_ROS_H

#include <navit_core/base_global_planner.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include "ipa_voronoi_explorator/VoronoiExploratorConfig.h"

namespace ipa_voronoi_explorator
{
class VoronoiExploratorRos : public navit_core::CoveragePlanner
{
public:
  using DynamicConfig = ipa_voronoi_explorator::VoronoiExploratorConfig;

  void initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override;

  bool makePlan(const nav_msgs::Path& edge_path, const geometry_msgs::Pose& start_pose,
                nav_msgs::Path& coverage_path) override;

private:
  void reconfigureCB(DynamicConfig& config, uint32_t level);

  void downsampleTrajectory(const std::vector<geometry_msgs::Pose2D>& path_uncleaned,
                            std::vector<geometry_msgs::Pose2D>& path, const double min_dist_squared);

  std::string name_;
  ros::NodeHandle node_handle_;
  std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ros_;
  dynamic_reconfigure::Server<DynamicConfig> dynamic_reconfigure_;
  struct Config
  {
    double robot_radius = 0.3;
    double coverage_radius = 1.0;

    // # map correction
    int map_correction_closing_neighborhood_size{};  // Applies a closing operation to neglect inaccessible areas and
                                                     // map errors/artifacts if the
                                                     // map_correction_closing_neighborhood_size parameter is larger
                                                     // than 0. # The parameter then specifies the iterations (or
                                                     // neighborhood size) of that closing operation. # int
  } config_;
};

}  // namespace ipa_voronoi_explorator

#endif  // IPA_VORONOI_EXPLORATOR_BOUSTROPHEDON_EXPLORATOR_ROS_H
