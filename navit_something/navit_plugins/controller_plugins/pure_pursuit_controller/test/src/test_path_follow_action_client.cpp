#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <navit_msgs/ComputePathAction.h>
#include <navit_msgs/FollowPathAction.h>
#include <navit_msgs/MoveBaseGoal.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <termios.h>
#include <thread>


namespace PathFollow
{
    const int NB_DISABLE = 0;
    const int NB_ENABLE = 1;

    typedef actionlib::SimpleActionClient<navit_msgs::FollowPathAction> ActionClient;

    class TestPathFollow{
    public: 
        TestPathFollow(const std::string client_name, bool flag=true) : ac_(client_name, flag) {};

        void start_follow();

    private:
        void init();

        void actionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const navit_msgs::FollowPathResultConstPtr& result);

        void pointCallback(const geometry_msgs::PointStamped& waypoint);

        void mapCallback(const nav_msgs::OccupancyGrid& global_map);

        void splineOptimize();

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

        inline float distance(const geometry_msgs::PointStamped& point1, const geometry_msgs::PointStamped& point2){
            float euclid_distance = sqrt((point1.point.x - point2.point.x) * (point1.point.x - point2.point.x) + 
                                  (point1.point.y -point2.point.y) * (point1.point.y - point2.point.y));
            return euclid_distance;                 
        }

        ros::NodeHandle nh_{"test_path_follow"};
        ros::NodeHandle pnh_{"~"};
        ros::Publisher path_pub = nh_.advertise<nav_msgs::Path>("planned_waypoint_path",10);
        ros::Publisher path_marker_pub = nh_.advertise<geometry_msgs::PolygonStamped>("path_marker_node", 10);
        ros::Timer pub_timer = nh_.createTimer(ros::Duration(0.1), &TestPathFollow::publish, this);
        ActionClient ac_;
        nav_msgs::Path path_;
        navit_msgs::FollowPathGoal goal_;
        visualization_msgs::Marker point_marker_;
        ros::Publisher point_marker_pub = nh_.advertise<visualization_msgs::Marker>("point_marker_node", 10);
        visualization_msgs::Marker path_marker_;
        nav_msgs::OccupancyGrid global_map_;
        std::unique_ptr<std::thread> thread_ptr_;
        const double_t point_size_ = 2;
        bool assemble_finish_ = false;

    };

    void TestPathFollow::init(){
        point_marker_.type = 7;
        point_marker_.scale.x = 0.7;
        point_marker_.scale.y = 0.7;
        point_marker_.scale.z = 0.0;

        point_marker_.color.r = 1.0f;
        point_marker_.color.g = 0.0f;
        point_marker_.color.b = 0.0f;
        point_marker_.color.a = 1.0;

        point_marker_.header.frame_id = "map";
        point_marker_.pose.position.x = 0.0;
        point_marker_.pose.position.y = 0.0;
    }

    void TestPathFollow::mapCallback(const nav_msgs::OccupancyGrid& global_map){
        global_map_ = global_map;
    }

    void TestPathFollow::actionDoneCallback(const actionlib::SimpleClientGoalState& state, 
                                            const navit_msgs::FollowPathResultConstPtr& result){

        ROS_INFO("Server Responds with State [%s]", state.toString().c_str());
    }

    void TestPathFollow::publish(const ros::TimerEvent&){
        path_.header.frame_id = "map";
        path_.header.stamp = ros::Time::now();
        path_pub.publish(path_);
        path_marker_pub.publish(path_marker_);
        // point_marker_pub.publish(point_marker_);
    }

    void TestPathFollow::pointCallback(const geometry_msgs::PointStamped& waypoint){
        ROS_INFO("Clicked point loaded.");
        auto carrot_msg = geometry_msgs::PoseStamped();
        auto prev_msg = geometry_msgs::PoseStamped();
        carrot_msg.header = waypoint.header;
        carrot_msg.pose.position.x = waypoint.point.x;
        carrot_msg.pose.position.y = waypoint.point.y;
        carrot_msg.pose.position.z = waypoint.point.z; 
        std::cout << "carrot_x: " << waypoint.point.x << std::endl;
        std::cout << "carrot_y: " << waypoint.point.y << std::endl;
        std::cout << "carrot_z: " << waypoint.point.z << std::endl;
        if (path_.poses.empty()) {
            // get new clicked point
            prev_msg.header = carrot_msg.header;
            prev_msg.pose.position.x = carrot_msg.pose.position.x;
            prev_msg.pose.position.y = carrot_msg.pose.position.y;
            prev_msg.pose.position.z = carrot_msg.pose.position.z;

            // give the path
            path_.poses.push_back(carrot_msg);
            path_marker_.points.push_back(waypoint.point);

            // give the points
            point_marker_.points.push_back(waypoint.point);
            point_marker_pub.publish(point_marker_);

        }else{
        // get previous clicked point
            prev_msg.header = waypoint.header;
            prev_msg.pose.position.x = path_.poses.back().pose.position.x;
            prev_msg.pose.position.y = path_.poses.back().pose.position.y;
            prev_msg.pose.position.z = path_.poses.back().pose.position.z; 

            // linear interpolation
            int interpolation_num = 50;
            int idx;
            float dxdy = (carrot_msg.pose.position.y - prev_msg.pose.position.y) / (carrot_msg.pose.position.x - prev_msg.pose.position.x);
            float dx = (carrot_msg.pose.position.x - prev_msg.pose.position.x) / (interpolation_num + 1);
            float dy = dx * dxdy;
            float inter_x, inter_y;

            for (idx = interpolation_num - 1; idx >= 0; idx--) {
                geometry_msgs::PoseStamped inter_msg;
                inter_x = prev_msg.pose.position.x + dx * (interpolation_num - idx);
                inter_y = prev_msg.pose.position.y + dy * (interpolation_num - idx);
                inter_msg.pose.position.x = inter_x;
                std::cout << "inter_x: " << inter_x << std::endl;
                inter_msg.pose.position.y = inter_y;
                std::cout << "inter_y: " << inter_y << std::endl;
                path_.poses.push_back(inter_msg);
            }
            // give the path
            path_.poses.push_back(carrot_msg);
            path_marker_.points.push_back(waypoint.point);
            
            // give the points
            point_marker_.points.push_back(waypoint.point);
            point_marker_pub.publish(point_marker_);

            ROS_INFO("POINTCALLBACK finished.");
            assemble_finish_ = true;
        }
    }
    void TestPathFollow::splineOptimize(){
        
    }

    void TestPathFollow::start_follow(){
        init();

        ROS_WARN("PLEASE click two points in RViz!!!");

        ros::Subscriber map_sub = nh_.subscribe("/map", 10, &TestPathFollow::mapCallback, this);

        ros::Subscriber marked_point_sub = nh_.subscribe("/clicked_point", 10, &TestPathFollow::pointCallback, this);

        thread_ptr_.reset(new std::thread(&TestPathFollow::terminalMonitor,this));

        ROS_INFO("Waiting for server .....");

        bool server_exists = ac_.waitForServer(ros::Duration(20.0));

        if(!server_exists){
            ROS_ERROR("Could not connect to server, holding.....");
        }else{
            ROS_INFO("Connect to PathFollow Action Server successfully.");
        }

        ros::spin();
    }

    void TestPathFollow::terminalMonitor(){
        int rate = 50;
        ros::Rate loop(rate);
        while(ros::ok()){
            if (assemble_finish_) {
                if (!path_.poses.empty()) {
                    goal_.path = path_;
                    ROS_INFO("Planned Path has been Loaded.");
                }else{
                    ROS_ERROR("\n Invalid path, please check the path!!!");
                }
                goal_.controller_plugin = "test_3";
                ac_.sendGoal(goal_, boost::bind(&TestPathFollow::actionDoneCallback, this, _1, _2));
            }
            // assemble_finish_ = false;
            loop.sleep();
        }
    }  
}  // namespace PathFollow

int main(int argc, char **argv){
    ros::init(argc, argv, "test_path_follow_node");

    PathFollow::TestPathFollow test_path_follow("/follow_path", true);

    test_path_follow.start_follow();

    return 0;
}
    
