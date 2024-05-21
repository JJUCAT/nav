#include "slic3r_coverage_planner/slic3r_coverage_planner.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <gtest/gtest.h>

// Test the Slic3rCoveragePlanner class, makePlan() method

//class Slic3rCoveragePlanner : public navit_core::CoveragePlanner {
//public:
//  Slic3rCoveragePlanner();
//  ~Slic3rCoveragePlanner();
//
//  //virtual void initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) = 0;
//  void initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override 
//  {
//    ROS_INFO("Slic3rCoveragePlanner::initialize");
//    initialize(name);
//  }
//
//  void initialize(const std::string& name);
//
//  bool makePlan(const geometry_msgs::Polygon& coverage_area,
//                const std::vector<geometry_msgs::Polygon>& holes,
//                const geometry_msgs::Pose& start,
//                const geometry_msgs::Pose& end,
//                std::vector<nav_msgs::Path>& coverage_path,
//                std::vector<nav_msgs::Path>& contour_path) override;
//private:
//  void traverse(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
//    for (auto &contour: contours) {
//        if (contour.children.empty()) {
//            line_groups.push_back(Polygons());
//        } else {
//            traverse(contour.children, line_groups);
//        }
//        line_groups.back().push_back(contour.polygon);
//    }
//}
//  double angle_, distance_, outer_offset_;
//  bool is_contour_;
//};
//
//} // namespace slic3r_coverage_planner
//

using namespace slic3r_coverage_planner;

class Slic3rCoveragePlannerFixture : public ::testing::Test {
    protected:
        void SetUp() override {
            // Code here will be called immediately after the constructor (right
           // before each test).
           // You can do set-up work for each test here.
            ros::NodeHandle nh;
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener(tf_buffer);
            tf2_ros::StaticTransformBroadcaster tf_broadcaster;
            tf2::Transform transform;
            transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped.header.stamp = ros::Time::now();
            transform_stamped.header.frame_id = "map";
            transform_stamped.child_frame_id = "base_link";
            transform_stamped.transform = tf2::toMsg(transform);
            tf_broadcaster.sendTransform(transform_stamped);

            costmap_ptr = std::make_shared<navit_costmap::Costmap2DROS>("costmap", tf_buffer, nh);

            planner.initialize("planner", costmap_ptr);

            // Create a coverage areas
            big_area.points.resize(4);
            big_area.points[0].x = 0.0;
            big_area.points[0].y = 0.0;
            big_area.points[1].x = 100.0;
            big_area.points[1].y = 0.0;
            big_area.points[2].x = 100.0;
            big_area.points[2].y = 100.0;
            big_area.points[3].x = 0.0;
            big_area.points[3].y = 100.0;

            // create a small area
            small_area.points.resize(4);
            small_area.points[0].x = 0.0;
            small_area.points[0].y = 0.0;
            small_area.points[1].x = 10.0;
            small_area.points[1].y = 0.0;
            small_area.points[2].x = 10.0;
            small_area.points[2].y = 10.0;
            small_area.points[3].x = 0.0;
            small_area.points[3].y = 10.0;

            // create a empty area
            empty_area.points.resize(0);
        }

        void TearDown() override {
            // Code here will be called immediately after each test (right
            // before the destructor).
            // You can do clean-up work that doesn't throw exceptions here.
            coverage_path.clear();
            contour_path.clear();
        }

        // Objects declared here can be used by all tests in the test suite for Foo.
        Slic3rCoveragePlanner planner;
        std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ptr;

        // test polygons
        geometry_msgs::Polygon big_area;
        geometry_msgs::Polygon small_area;
        geometry_msgs::Polygon empty_area;
        
        std::vector<geometry_msgs::Polygon> holes;
        geometry_msgs::Pose start_pose;
        geometry_msgs::Pose end_pose;
        std::vector<nav_msgs::Path> coverage_path;
        std::vector<nav_msgs::Path> contour_path;

};


TEST_F(Slic3rCoveragePlannerFixture, bigArea)
{

  auto result = planner.makePlan(big_area, holes, start_pose, end_pose, coverage_path, contour_path);

  EXPECT_TRUE(result);
}

TEST_F(Slic3rCoveragePlannerFixture, smallArea)
{

  auto result = planner.makePlan(small_area, holes, start_pose, end_pose, coverage_path, contour_path);

  EXPECT_TRUE(result);
}

TEST_F(Slic3rCoveragePlannerFixture, emptyArea)
{

  auto result = planner.makePlan(empty_area, holes, start_pose, end_pose, coverage_path, contour_path);

  EXPECT_EQ(result, false);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_slic3r_coverage_planner");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
