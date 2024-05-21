/*
* @Author: czk
* @Date:   2022-09-24 11:13:53
* @Last Modified by:   chenzongkui
* @Last Modified time: 2022-11-18 22:25:21
*/
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <navit_msgs/ComputePathAction.h>

#include <navit_msgs/CoveragePathAction.h>

#include <navit_msgs/MoveBaseGoal.h>

#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PolygonStamped.h>

#include <geometry_msgs/Point32.h>

#include <geometry_msgs/PointStamped.h>

#include <std_msgs/Int32.h>

#include <nav_msgs/OccupancyGrid.h>

#include <termios.h>

#include <thread>

namespace Test {

const int NB_DISABLE = 0;
const int NB_ENABLE = 1;


typedef actionlib::SimpleActionClient<navit_msgs::CoveragePathAction> ActionClient;

class TestCCPP {
public:
    TestCCPP(const std::string client_name, bool flag = true) : action_client_(client_name, flag) {};
    // ~TestCCPP();
    void run();
private:
    void init();

    void actionDoneCallBack(const actionlib::SimpleClientGoalState& state,
                            const navit_msgs::CoveragePathResultConstPtr& result);

    void goalCallBack(const geometry_msgs::PointStampedConstPtr& msg);

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

    void terminalMonitor();

    void publish(const ros::TimerEvent&);

    inline int kbhit() {
        struct timeval tv;
        fd_set fds;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);  // STDIN_FILENO is 0
        select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);

        return FD_ISSET(STDIN_FILENO, &fds);
    }

    inline void nonblock(int state) {
        struct termios ttystate;
        // get the terminal state
        tcgetattr(STDIN_FILENO, &ttystate);

        if (state == NB_ENABLE) {
            // turn off canonical mode
            ttystate.c_lflag &= ~ICANON;
            // minimum of number input read.
            ttystate.c_cc[VMIN] = 1;
        } else if (state == NB_DISABLE) {
            // turn on canonical mode
            ttystate.c_lflag |= ICANON;
        }
        // set the terminal attributes.
        tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    }

    inline float distance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
        float dis = sqrt((pose1.position.x - pose2.position.x) * (pose1.position.x - pose2.position.x) +
                         (pose1.position.y - pose2.position.y) * (pose1.position.y - pose2.position.y));
        return dis;
    }
    ros::NodeHandle nh_{"test_ccpp"};

    ros::NodeHandle nh_p_{"~"};

    ros::Publisher edge_path_pub_ = nh_.advertise<nav_msgs::Path>("edge_path", 8);

    ros::Publisher edge_path_marker_node_pub_ = nh_.advertise<visualization_msgs::Marker>("edge_path_node_marker", 8);

    ros::Publisher first_node_hit_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("firt_node_marker_hit_zone", 8);

    ros::Publisher result_coverage_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("coverage_path_marker", 8);

    ros::Publisher result_wall_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("wall_path_marker", 8);

    ros::Timer pub_timer = nh_.createTimer(ros::Duration(0.1), &TestCCPP::publish, this);

    ActionClient action_client_;

    nav_msgs::Path edge_path_;

    visualization_msgs::Marker node_maker_;

    geometry_msgs::PolygonStamped hit_zone_;

    std::unique_ptr<std::thread> thread_ptr_;

    navit_msgs::CoveragePathGoal goal_;

    nav_msgs::OccupancyGrid map_;

    bool assemble_finished_ = false;
};
void TestCCPP::init() {

    node_maker_.type = 7;
    node_maker_.scale.x = 0.7;
    node_maker_.scale.y = 0.7;
    node_maker_.scale.z = 0.0;

    node_maker_.color.r = 1.0f;
    node_maker_.color.g = 0.0f;
    node_maker_.color.b = 0.0f;
    node_maker_.color.a = 1.0;

    node_maker_.header.frame_id = "map";

    node_maker_.pose.position.x = 0;
    node_maker_.pose.position.y = 0;

}
void TestCCPP::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    map_ = *msg;
}

void TestCCPP::actionDoneCallBack(const actionlib::SimpleClientGoalState& state,
                                  const navit_msgs::CoveragePathResultConstPtr& result) {
    visualization_msgs::MarkerArray coverage_path_marker, wall_path_marker;

    for (int i = 0; i < result->coverage_path.poses.size(); i++) {
        visualization_msgs::Marker maker;
        maker.header.frame_id = "map";
        maker.type = 0;
        maker.color.r = 1.0f;
        maker.color.g = 0.0f;
        maker.color.b = 0.0f;
        maker.color.a = 1.0;

        maker.scale.x = 0.05;
        maker.scale.y = 0.05;
        maker.scale.z = 0.0;
        maker.id = i;
        maker.pose.position.x = result->coverage_path.poses[i].pose.position.x;
        maker.pose.position.y = result->coverage_path.poses[i].pose.position.y;
        maker.pose.position.z = result->coverage_path.poses[i].pose.position.z;

        maker.pose.orientation.x = result->coverage_path.poses[i].pose.orientation.x;
        maker.pose.orientation.y = result->coverage_path.poses[i].pose.orientation.y;
        maker.pose.orientation.z = result->coverage_path.poses[i].pose.orientation.z;
        maker.pose.orientation.w = result->coverage_path.poses[i].pose.orientation.w;

        coverage_path_marker.markers.push_back(maker);
    }

       for (int i = 0; i < result->wall_path.poses.size(); i++) {
        visualization_msgs::Marker maker;
        maker.header.frame_id = "map";
        maker.type = 0;
        maker.color.r = 1.0f;
        maker.color.g = 0.0f;
        maker.color.b = 0.0f;
        maker.color.a = 1.0;

        maker.scale.x = 0.05;
        maker.scale.y = 0.05;
        maker.scale.z = 0.0;
        maker.id = i;
        maker.pose.position.x = result->wall_path.poses[i].pose.position.x;
        maker.pose.position.y = result->wall_path.poses[i].pose.position.y;
        maker.pose.position.z = result->wall_path.poses[i].pose.position.z;

        maker.pose.orientation.x = result->wall_path.poses[i].pose.orientation.x;
        maker.pose.orientation.y = result->wall_path.poses[i].pose.orientation.y;
        maker.pose.orientation.z = result->wall_path.poses[i].pose.orientation.z;
        maker.pose.orientation.w = result->wall_path.poses[i].pose.orientation.w;

        wall_path_marker.markers.push_back(maker);
    }
    result_coverage_path_pub_.publish(coverage_path_marker);
    result_wall_path_pub_.publish(wall_path_marker);

    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
}

void TestCCPP::publish(const ros::TimerEvent&) {
    edge_path_.header.frame_id = "map";
    edge_path_.header.stamp = ros::Time::now();
    edge_path_pub_.publish(edge_path_);

    edge_path_marker_node_pub_.publish(node_maker_);
}

void TestCCPP::goalCallBack(const geometry_msgs::PointStampedConstPtr& msg) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = msg->point.x;
    pose.pose.position.y = msg->point.y;

    edge_path_.poses.push_back(pose);

    visualization_msgs::Marker marker;
    node_maker_.points.push_back(msg->point);

    if (edge_path_.poses.size() >= 3 && distance(edge_path_.poses.back().pose, edge_path_.poses.front().pose) < 0.5) {
        edge_path_.poses.pop_back();
        edge_path_.poses.push_back(edge_path_.poses.front());
        node_maker_.points.pop_back();
        node_maker_.points.push_back(node_maker_.points.front());
        assemble_finished_ = true;
    }
}

void TestCCPP::run() {
    init();

    ros::Subscriber marker_goal_sub_ = nh_.subscribe("/clicked_point", 8, &TestCCPP::goalCallBack, this);

    ros::Subscriber map_sub_ = nh_.subscribe("/map", 8, &TestCCPP::mapCallback, this);

    thread_ptr_.reset(new std::thread(&TestCCPP::terminalMonitor, this));

    ROS_INFO("Waiting for server... ");

    bool server_exists = action_client_.waitForServer(ros::Duration(10.0)); //wait for 10s.

    if (!server_exists) {
        ROS_ERROR("Could not connect to server, halting.");
        //return 0;
    } else  {
        ROS_INFO("Connected to action server successfully.");
    }

    ros::spin();
}

// For get terminal status thread
void TestCCPP::terminalMonitor() {
  int hz = 100;
  ros::Rate loop_rate(hz);
  while (ros::ok()) {
    nonblock(NB_ENABLE);
    if (kbhit()) {
      std_msgs::Int32 msg;
      msg.data = getchar();

      if (msg.data >= 32 && msg.data <= 122) {
        switch(msg.data) {
            case ' ':
                printf("\n输入 q: 取消上一个点\n输入 x: 生成全覆盖轨迹\n");
                break;
            case 'q':
                if (edge_path_.poses.size() != 0 ) {
                    edge_path_.poses.pop_back();
                } else {
                    ROS_WARN("\nThere are no point left. Please insert new point..");
                }
                if (node_maker_.points.size() != 0) {
                    node_maker_.points.pop_back();
                } else {
                    ROS_WARN("\nThere are no point left. Please insert new point..");
                }
                break;
            case 'e':
                if (assemble_finished_) {
                    goal_.map = map_;
                    std::cout << "size size is is " << goal_.map.data.size() << std::endl;
                    goal_.edge_path = edge_path_;
                    goal_.planner_plugin = "test_1";
                    action_client_.sendGoal(goal_, boost::bind(&TestCCPP::actionDoneCallBack, this, _1, _2));
                } else {
                    ROS_ERROR("\nEdge path is invalid, please check the path.");
                }
                break;
            default:
                break;
        }
      }
    }
    nonblock(NB_DISABLE);
    loop_rate.sleep();
  }
}
} // namespace Test

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_ccpp_action_client_node");

    Test::TestCCPP test_cpp("/coverage_path", true);

    test_cpp.run();

    return 0;
}
