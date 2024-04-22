#ifndef SERVER_HELPER_H
#define SERVER_HELPER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <navit_costmap/costmap_2d_ros.h>

#include <navit_common/geometry_algorithms.h>
#include <navit_common/path_handle.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/SE2StateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace navit_planner
{

class PlannerServerHelper
{
    public:
        using Ptr = std::shared_ptr<PlannerServerHelper>;

        PlannerServerHelper(){};
        ~PlannerServerHelper(){};

        bool getCoveragePathLength(const std::vector<nav_msgs::Path> coverage_paths,
                                   const std::vector<nav_msgs::Path> contour_paths,
                                   float &length_path) {
            for (auto& path : coverage_paths) {
                if (path.poses.size() < 2) {
                    ROS_ERROR("Coverage path size is less than 2");
                    return false;
                }
                length_path += getPathLength(path);
            }

            for (auto& path : contour_paths) {
                if (path.poses.size() < 2) {
                    ROS_ERROR("Contour path size is less than 2");
                    return false;
                }
                length_path += getPathLength(path);
            }

            return true;
        }

        float getPathLength(const nav_msgs::Path path) {
            float length = 0.0;

            for(auto i = 0; i < path.poses.size() - 1; i++) {
                double diff_x = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
                double diff_y = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
                length += std::hypot(diff_x, diff_y);
            }
            length += std::hypot(path.poses[path.poses.size()-2].pose.position.x - path.poses[path.poses.size()-1].pose.position.x,
                                 path.poses[path.poses.size()-2].pose.position.y - path.poses[path.poses.size()-1].pose.position.y);

            return length;
        }

        bool getPolygonWithHolesArea(const geometry_msgs::Polygon coverage_area,
                                      const std::vector<geometry_msgs::Polygon> holes,
                                      float &area) {

            Polygon boost_coverage_area = convertToBoostPolygon(coverage_area);
            std::vector<Polygon> boost_holes = convertToBoostPolygons(holes);

            for (int i = 0; i < boost_holes.size(); i++) {
                boost::geometry::correct(boost_holes[i]);
            }
            area = navit_common::geometry::getPolygonWithHolesArea(boost_coverage_area, boost_holes);
            return true;
        }

        Polygon convertToBoostPolygon(const geometry_msgs::Polygon& ros_polygon) {
            Polygon boost_polygon;
            for(const auto& point : ros_polygon.points) {
                boost::geometry::append(boost_polygon, Point(point.x, point.y));
            }
            boost::geometry::append(boost_polygon, Point(ros_polygon.points.front().x, ros_polygon.points.front().y));
            // boost::geometry::reverse(boost_polygon);
            // boost::geometry::correct(boost_polygon);
            return boost_polygon;
        }

        geometry_msgs::Polygon convertToRosPolygon (const Polygon& boost_polygon) {
            geometry_msgs::Polygon ros_polygon;

            for(const auto& point : boost_polygon.outer()) {
                geometry_msgs::Point32 ros_point;
                ros_point.x = point.x();
                ros_point.y = point.y();
                ros_polygon.points.push_back(ros_point);
            }
            return ros_polygon;
        }
        std::vector<Polygon> convertToBoostPolygons(const std::vector<geometry_msgs::Polygon>& ros_polygons) {
            std::vector<Polygon> boost_polygons;
            for(const auto& ros_polygon : ros_polygons) {
                boost_polygons.push_back(convertToBoostPolygon(ros_polygon));
            }
            return boost_polygons;
        }

        bool isPoseInPolygons(const geometry_msgs::Pose& pose, const std::vector<geometry_msgs::Polygon>& polygons)
        {
            std::vector<Polygon> boost_polygons = convertToBoostPolygons(polygons);
            Point boost_point(pose.position.x, pose.position.y);
            return navit_common::geometry::isPoseInPolygons(boost_point, boost_polygons);
        }

        bool isPoseInPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Polygon& polygon)
        {
            Polygon boost_polygon = convertToBoostPolygon(polygon);
            boost::geometry::correct(boost_polygon);
            Point boost_point(pose.position.x, pose.position.y);
            return navit_common::geometry::isPoseInPolygon(boost_point, boost_polygon);
        }

        bool adjustPolygonBoundary(const geometry_msgs::Polygon& coverage_area,
                                   const float expand_distance,
                                   geometry_msgs::Polygon& adjusted_coverage_area) {
            Polygon boost_coverage_area = convertToBoostPolygon(coverage_area);
            boost::geometry::correct(boost_coverage_area);
            Polygon boost_adjusted_coverage_area = navit_common::geometry::adjustPolygonBoundary(boost_coverage_area, -expand_distance);

            std::cout << "boost_adjusted_coverage_area.outer().size() is " << boost_adjusted_coverage_area.outer().size() << std::endl;

            if (boost_adjusted_coverage_area.outer().size() < 3 ) {
                std::cout << "boost_adjusted_coverage_area.outer().size() is " << boost_adjusted_coverage_area.outer().size() << std::endl;
                return false;
            }
            adjusted_coverage_area = convertToRosPolygon(boost_adjusted_coverage_area);
            return true;
        }

        bool adjustPolygonsBoundary(const std::vector<geometry_msgs::Polygon>& holes,
                                    const float expand_distance,
                                    std::vector<geometry_msgs::Polygon>& adjusted_holes) {
            std::vector<Polygon> boost_holes = convertToBoostPolygons(holes);
            for (int i = 0; i < boost_holes.size(); i++) {
                boost::geometry::correct(boost_holes[i]);
            }
            std::vector<Polygon> boost_adjusted_holes = navit_common::geometry::adjustPolygonsBoundary(boost_holes, expand_distance);

            for (auto& boost_adjusted_hole : boost_adjusted_holes) {
                geometry_msgs::Polygon ros_adjusted_hole = convertToRosPolygon(boost_adjusted_hole);
                adjusted_holes.push_back(ros_adjusted_hole);
            }
            return true;
        }


        bool simplifyPolygon(const geometry_msgs::Polygon& input,
                             const float tolerance,
                             geometry_msgs::Polygon& output) {
            Polygon boost_input = convertToBoostPolygon(input);

            Polygon boost_output = navit_common::geometry::simplifyPolygon(boost_input, tolerance);
            output = convertToRosPolygon(boost_output);

            return true;
        }

        bool simplifyPolygons(const std::vector<geometry_msgs::Polygon>& input_polygons,
                              const double tolerance,
                              std::vector<geometry_msgs::Polygon>& output_polygons) {
            for (auto& input : input_polygons) {
                Polygon boost_input = convertToBoostPolygon(input);
                Polygon output = navit_common::geometry::simplifyPolygon(boost_input, tolerance);
                geometry_msgs::Polygon ros_output;
                ros_output = convertToRosPolygon(output);
                output_polygons.push_back(ros_output);
            }
            return true;
        }
    private:
};
class OmplRos {
public:
    using Ptr = std::shared_ptr<OmplRos>;

    OmplRos(ros::NodeHandle nh,
            const std::shared_ptr<tf2_ros::Buffer>& tf) : initialized_(false),
            bounds_(2),
            reeds_shepp_statespace_(new ompl::base::ReedsSheppStateSpace(0.8)),
            dubins_state_space_(new ompl::base::DubinsStateSpace(0.55, false)),
            rs_simple_setup_(new ompl::geometric::SimpleSetup(reeds_shepp_statespace_)),
            dubins_simple_setup_(new ompl::geometric::SimpleSetup(dubins_state_space_))
    {
        tf_ = tf;
        planner_server_helper_ptr_ = std::make_shared<PlannerServerHelper>();
        initialize(nh, tf);
    };

    ~OmplRos() {};
    void initialize(const ros::NodeHandle pnh,
                    // std::shared_ptr<navit_costmap::Costmap2DROS>  costmap_ros_ptr,
                    const std::shared_ptr<tf2_ros::Buffer>& tf)
    {
        // costmap_ros_ptr_ = costmap_ros_ptr;
        pnh.param("min_turning_radius", settings_.min_turning_radius, 0.1);
        pnh.param("max_planning_duration", settings_.max_planning_duration, 0.2);
        pnh.param<int>("valid_state_max_cost", settings_.valid_state_max_cost, 252);
        pnh.param<int>("interpolation_num_poses", settings_.interpolation_num_poses, 50);
        pnh.param<bool>("allow_unknown", settings_.allow_unknown, false);
        pnh.param<int>("skip_poses", settings_.skip_poses, 0);
        pnh.param<bool>("display_planner_output", settings_.display_planner_output, false);

        initialized_ = true;
    }

    void state2Pose(const ob::State* state, geometry_msgs::PoseStamped& pose, const bool use_position_z)
    {
        const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
        pose.pose.position.x = s->getX();
        pose.pose.position.y = s->getY();
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
        pose.header.frame_id = robot_frame_;
        pose.header.stamp = stamp_;
        if (use_position_z) {
            pose.pose.position.z = 0.2;
        } else {
            pose.pose.position.z = 0.0;
        }
    }

    void pose2State(const geometry_msgs::Pose& pose, ob::State* state)
    {
        ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
        s->setX(pose.position.x);
        s->setY(pose.position.y);
        s->setYaw(tf::getYaw(pose.orientation));
    }

    bool setBoundingBox(const geometry_msgs::Polygon& drivable_area,
                        const std::vector<geometry_msgs::Polygon>& undrivable_areas)
    {
        boost_drivable_area_ = planner_server_helper_ptr_->convertToBoostPolygon(drivable_area);
        boost_undrivable_areas_ = planner_server_helper_ptr_->convertToBoostPolygons(undrivable_areas);

        return true;
    }

    bool planPath( const geometry_msgs::Pose& start_pose,
                   const geometry_msgs::Pose& goal_pose,
                   std::vector<geometry_msgs::PoseStamped>& path_poses,
                   bool use_dubins,
                   bool use_position_z) {

        if (!initialized_)
        {
            ROS_ERROR("Planner not initialized!");
            return false;
        }
        ob::RealVectorBounds bounds(2);
        // TODO(czk): use costmap or polygon with holes to set the boundary.
        // this is the boundary of the map, we should get it from costmap, but now we just set it manually(czk)
        bounds.setLow(0, -200.0);
        bounds.setHigh(0, 200.0);
        bounds.setLow(1, -200.0);
        bounds.setHigh(1, 200.0);

        // create start and goal states
        geometry_msgs::Pose local_startpose, local_goalpose;

        if (use_dubins) {
            ROS_INFO("Use dubins...");
            ompl::base::ScopedState<> start(dubins_state_space_);
            ompl::base::ScopedState<> goal(dubins_state_space_);

            ompl::base::SpaceInformationPtr si(dubins_simple_setup_->getSpaceInformation());
            dubins_state_space_->as<ob::DubinsStateSpace>()->setBounds(bounds);
            dubins_simple_setup_->setStateValidityChecker([this, si](const ob::State *state){return isStateValid(state, boost_drivable_area_, boost_undrivable_areas_);});

            local_startpose = start_pose;
            local_goalpose = goal_pose;

            pose2State(local_startpose, start());
            pose2State(local_goalpose, goal());

            dubins_simple_setup_->clear();
            dubins_simple_setup_->setStartAndGoalStates(start, goal);

            if (!dubins_simple_setup_->solve(settings_.max_planning_duration)) {
                return false;
            }  else {
                ROS_INFO("[dubins_planner] Valid plan found");
            }

            dubins_simple_setup_->simplifySolution();
            ompl::geometric::PathGeometric path = dubins_simple_setup_->getSolutionPath();

            path.interpolate(settings_.interpolation_num_poses);

            if (path.getStateCount() > settings_.interpolation_num_poses) {
                return false;
            }

            path_poses.resize(path.getStateCount());

            for (unsigned int i = 0; i < path.getStateCount(); i++)
            {
                const ompl::base::State* state = path.getState(i);
                state2Pose(state, path_poses[i], use_position_z);
                path_poses[i].header.frame_id = robot_frame_;
                path_poses[i].header.stamp = ros::Time::now();
            }

        } else {
            ROS_INFO("Use RS...");
            ompl::base::ScopedState<> start(reeds_shepp_statespace_);
            ompl::base::ScopedState<> goal(reeds_shepp_statespace_);
            ompl::base::SpaceInformationPtr si(rs_simple_setup_->getSpaceInformation());

            reeds_shepp_statespace_->as<ob::ReedsSheppStateSpace>()->setBounds(bounds);
            rs_simple_setup_->setStateValidityChecker([this, si](const ob::State *state){return isStateValid(state, boost_drivable_area_, boost_undrivable_areas_);});

            local_startpose = start_pose;
            local_goalpose = goal_pose;

            // convert start and goal poses to ompl base states
            pose2State(local_startpose, start());
            pose2State(local_goalpose, goal());

            rs_simple_setup_->clear();
            rs_simple_setup_->setStartAndGoalStates(start, goal);

            if (!rs_simple_setup_->solve(settings_.max_planning_duration)) {
                return false;
            }  else {
                ROS_DEBUG("[reeds_shepp_planner] Valid plan found");
            }

            rs_simple_setup_->simplifySolution();

            ompl::geometric::PathGeometric path = rs_simple_setup_->getSolutionPath();
            path.interpolate(settings_.interpolation_num_poses);

            if (path.getStateCount() > settings_.interpolation_num_poses) {
                return false;
            }

            path_poses.resize(path.getStateCount());

            for (unsigned int i = 0; i < path.getStateCount(); i++)
            {
                const ompl::base::State* state = path.getState(i);
                state2Pose(state, path_poses[i], use_position_z);
                path_poses[i].header.frame_id = robot_frame_;
                path_poses[i].header.stamp = ros::Time::now();
            }

            return true;
        }
        return true;
    }

    /*
    * @brief Checks if a state is valid
    * TODO(czk): 考虑到我们目前使用的polygon with holes的方式，这里我并没有使用costmap做碰撞检查的方式，而是直接使用了if point in polygon，我这样应该可以大大减少计算量
    * 如果后面有机会 ^_^ ，可以两种方式都实现一下
    */
    bool isStateValid(const ob::State *state,
                      const Polygon& boost_drivable,
                      const std::vector<Polygon>& boost_undrivable_areas) {

        const auto *reedsSheppState = state->as<ob::ReedsSheppStateSpace::StateType>();

        double x = reedsSheppState->getX();
        double y = reedsSheppState->getY();

        if (!navit_common::geometry::isPoseInPolygon(Point{x, y}, boost_drivable)) {
            ROS_ERROR("State is not in drivable area");
            return false;
        }

        for (const auto& boost_undrivable_area : boost_undrivable_areas) {
            if (navit_common::geometry::isPoseInPolygon(Point{x, y}, boost_undrivable_area)) {
                ROS_ERROR("State is in undrivable area");
                return false;
            }
        }
        return true;
    }

    // bool isStateValid::isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State *state)
    // {
    //     // check if state is inside boundary
    //     if (!si->satisfiesBounds(state)) {
    //         return false;
    //     }

    //     // TODO(czk): 考虑到我们目前使用的polygon with holes的方式，这里我并没有使用costmap做碰撞检查的方式，而是直接使用了if point in polygon，我这样应该可以大大减少计算量
    //     // 如果后面有机会 ^_^ ，可以两种方式都实现一下
    //     const ompl::base::SE2StateSpace::StateType *s =
    //     state->as<ompl::base::SE2StateSpace::StateType>();

    //     geometry_msgs::PoseStamped statePose;
    //     state2pose(s, statePose);

    //     if (fabs(s->getX()) < 5e-2 && fabs(s->getY()) < 5e-2)
    //     return true;

    //     transform(statePose, statePose, globalFrame_);

    //     uint8_t cost = costmapModel_->footprintCost(
    //     statePose.pose.position.x, statePose.pose.position.y,
    //     tf::getYaw(statePose.pose.orientation), footprint_);

    //     // check if state is in collision
    //     if (cost > validStateMaxCost_ && cost < 256 - (allowUnknown_?1:0))
    //     return false;

    //     return true;
    // }
private:
    std::shared_ptr<tf2_ros::Buffer> tf_;
    navit_costmap::Costmap2D* costmap_ptr_;
    std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ros_ptr_;
    bool initialized_ = false;

    ob::StateSpacePtr reeds_shepp_statespace_, dubins_state_space_;
    og::SimpleSetupPtr rs_simple_setup_;
    og::SimpleSetupPtr dubins_simple_setup_;
    ob::RealVectorBounds bounds_;

    // base_local_planner::CostmapModel* // ;
    std::vector<geometry_msgs::Point> footprint_;

    std::string robot_frame_ = "map";
    ros::Time stamp_;

    struct Settings {
        double min_turning_radius;
        double max_planning_duration;
        int valid_state_max_cost;
        int interpolation_num_poses;
        bool allow_unknown;
        int skip_poses;
        bool display_planner_output;
    } settings_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
    std::shared_ptr<PlannerServerHelper> planner_server_helper_ptr_;
    geometry_msgs::Polygon drivable_area_;
    std::vector<geometry_msgs::Polygon> undrivable_areas_;
    Polygon boost_drivable_area_;
    std::vector<Polygon> boost_undrivable_areas_;
};
} // navit_planner
#endif
