#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navit_msgs/CoveragePathOnPWHAction.h>
#include <std_msgs/String.h>
#include <google/protobuf/util/json_util.h>

#include "message_navit_map.pb.h"

namespace navit_test {
class FcppServerTest {
public:
    FcppServerTest() : ac_("/polygon_coverage_path", true) {
        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
        ROS_INFO("Action server started, sending goal.");
    }

    ~FcppServerTest() {};
    void run();

private:
    void mapCallback(const std_msgs::String::ConstPtr& msg);
    
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    navit::protocol::map_info::MapInfo sub_map_info_;
    actionlib::SimpleActionClient<navit_msgs::CoveragePathOnPWHAction> ac_;
};

void FcppServerTest::mapCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received map from map_server...");
    std::string json_string = msg->data;
    google::protobuf::util::JsonParseOptions options;
    options.ignore_unknown_fields = true;
    
    google::protobuf::util::JsonStringToMessage(json_string, &sub_map_info_, options);

    std::cout << "json_string = " << json_string << std::endl;
    ROS_INFO("Received map from map_server...");

    navit_msgs::CoveragePathOnPWHGoal goal;

    for (const auto& area : sub_map_info_.map_areas()) {
        if (area.type() == navit::protocol::map_info::MapArea::AREA_TYPE_FULL_COVERAGE) {
            geometry_msgs::Polygon coverage_area;
            geometry_msgs::Point32 point;

            for (const auto& pt : area.path()) {
                point.x = pt.x();
                point.y = pt.y();
                coverage_area.points.push_back(point);
            }
            
            goal.coverage_area = coverage_area;
            break; 
        }
    }

    for (const auto& area : sub_map_info_.map_areas()) {
        if (area.type() == navit::protocol::map_info::MapArea::AREA_TYPE_FORBIDDEN_AREA) {
            geometry_msgs::Polygon hole;
            geometry_msgs::Point32 point;

            for (const auto& pt : area.path()) {
                point.x = pt.x();
                point.y = pt.y();
                hole.points.push_back(point);
            }
            
            goal.holes.push_back(hole);
        }
    }

    goal.plugin_name = "fcpp";

    geometry_msgs::Pose start_pose;
    start_pose.position.x = 0.0;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.0;
    start_pose.orientation.w = 1.0;

    goal.start = start_pose;

    geometry_msgs::Pose end_pose;
    end_pose.position.x = 0.0;
    end_pose.position.y = 0.0;
    end_pose.position.z = 0.0;
    end_pose.orientation.w = 1.0;

    
    ROS_INFO("Waiting for action server to start.");
    ac_.sendGoal(goal);
}


void FcppServerTest::run() {
    map_sub_ = nh_.subscribe("/navit/map_info_update", 1, &FcppServerTest::mapCallback, this);
    ros::spin();
}


} // namespace navit_test

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coverage_planner_client");
    
    navit_test::FcppServerTest test;

    test.run();

    return 0;
}