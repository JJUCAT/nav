//
// Created by fan on 23-8-7.
//

#ifndef IPA_BOUSTROPHEDON_EXPLORATOR_BOUSTROPHEDON_EXPLORATOR_ROS_H
#define IPA_BOUSTROPHEDON_EXPLORATOR_BOUSTROPHEDON_EXPLORATOR_ROS_H

#include <navit_core/base_global_planner.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "ipa_boustrophedon_explorator/boustrophedon_explorator.h"
#include "ipa_boustrophedon_explorator/BoustrophedonExploratorConfig.h"

namespace ipa_boustrophedon_explorator
{
class BoustrophedonExploratorRos : public navit_core::CoveragePlanner
{
public:
  using BoustrophedonExploratorConfig = ipa_boustrophedon_explorator::BoustrophedonExploratorConfig;

  void initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override;

  bool makePlan(const nav_msgs::Path& edge_path, const geometry_msgs::Pose& start_pose,
                nav_msgs::Path& coverage_path) override;

  void reconfigureCB(BoustrophedonExploratorConfig& config, uint32_t level);

private:
  std::string name_;
  ros::NodeHandle node_handle_;
  std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ros_;
  dynamic_reconfigure::Server<BoustrophedonExploratorConfig> dynamic_reconfigure_;

  struct Config
  {
    int boustrophedon_exploration_algorithm{};  // 2 = boustrophedon explorator
                                                // 8 = boustrophedon variant explorator

    double robot_radius = 0.3;
    double coverage_radius = 1.0;

    // # map correction
    int map_correction_closing_neighborhood_size{};  // Applies a closing operation to neglect inaccessible areas and
                                                     // map errors/artifacts if the
                                                     // map_correction_closing_neighborhood_size parameter is larger
                                                     // than 0. # The parameter then specifies the iterations (or
                                                     // neighborhood size) of that closing operation. # int

    // parameters specific for the boustrophedon explorator
    double min_cell_area{};          // minimal area a cell can have, when using the boustrophedon explorator
    double path_eps{};               // the distance between points when generating a path
    double grid_obstacle_offset{};   // in [m], the additional offset of the grid to obstacles, i.e. allows to displace
                                     // the grid by more than the standard half_grid_size from obstacles
    int max_deviation_from_track{};  // in [pixel], maximal allowed shift off the ideal boustrophedon track to both
                                     // sides for avoiding obstacles on track setting
                                     // max_deviation_from_track=grid_spacing is usually a good choice for negative
                                     // values (e.g. max_deviation_from_track: -1) max_deviation_from_track is
                                     // automatically set to grid_spacing
    int cell_visiting_order{};       // cell visiting order
                                     //   1 = optimal visiting order of the cells determined as TSP problem
                                     //   2 = alternative ordering from left to right (measured on y-coordinates of the
    //   cells), visits the cells in a more obvious fashion to the human observer (though it
    //   is not optimal)

  } config_;

  BoustrophedonExplorer boustrophedon_explorer_;  // object that uses the boustrophedon exploration method to plan a
                                                  // path trough the room

  BoustrophedonVariantExplorer boustrophedon_variant_explorer_;  // object that uses the boustrophedon variant
                                                                 // exploration method to plan a path trough the room
};

}  // namespace ipa_boustrophedon_explorator

#endif  // IPA_BOUSTROPHEDON_EXPLORATOR_BOUSTROPHEDON_EXPLORATOR_ROS_H
