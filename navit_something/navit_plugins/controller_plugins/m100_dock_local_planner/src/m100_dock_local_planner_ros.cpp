#include <m100_dock_local_planner/m100_dock_local_planner_ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(m100_dock_local_planner::M100DockLocalPlannerROS, navit_core::Controller)

namespace m100_dock_local_planner {

M100DockLocalPlannerROS::M100DockLocalPlannerROS() : initialized_(false), goal_reached_(false), dis_old_(0.0) {}

M100DockLocalPlannerROS::~M100DockLocalPlannerROS() {}

void M100DockLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, navit_costmap::Costmap2DROS* costmap_ros) {
    if (!isInitialized()) {
        global_frame_ = "map";
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("kp", kp_, 0.1);
        private_nh.param("max_vel", max_vel_, 0.2);
        private_nh.param("min_vel", min_vel_, 0.05);
        private_nh.param("global_frame", global_frame_, global_frame_);
        private_nh.param("tolerance", tolerance_, 0.01);
        tf_          = tf;
        costmap_ros_ = costmap_ros;
        kp_          = fabs(kp_);
        max_vel_     = fabs(max_vel_);
        min_vel_     = fabs(min_vel_);
        initialized_ = true;
        ROS_INFO("[m100_dock_local_planner]: init over");
    } else
        ROS_WARN("[m100_dock_local_planner]: this planner has already init");
}

bool M100DockLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (!isInitialized()) {
        ROS_ERROR("[m100_dock_local_planner]: setPlan init failed!");
        return false;
    }
    ROS_INFO("[dock_local_planner]: Receive a new plan");
    std::string plan_id;
    global_plan_.clear();
    for (int i = 0; i < orig_global_plan.size(); i++) {
        if (orig_global_plan[i].pose.position.z < -1.0)
            continue;
        geometry_msgs::PoseStamped thisPoint;
        thisPoint.header = orig_global_plan[i].header;
        thisPoint.pose.position.x  = orig_global_plan[i].pose.position.x;
        thisPoint.pose.position.y  = orig_global_plan[i].pose.position.y;
        thisPoint.pose.position.z  = orig_global_plan[i].pose.position.z;
        thisPoint.pose.orientation = orig_global_plan[i].pose.orientation;
        global_plan_.push_back(thisPoint);
        if (i == 0)
            plan_id += std::to_string(-1) + " -> ";
        else if (i == orig_global_plan.size() - 1)
            plan_id += std::to_string(thisPoint.pose.position.z);
        else
            plan_id += std::to_string(thisPoint.pose.position.z) + " -> ";
    }
    ROS_INFO("[dock_local_planner]: Global plan id = %s \n", plan_id.c_str());
    return true;
}

bool M100DockLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!isInitialized()) {
        ROS_ERROR("[m100_dock_local_planner]: computeVel init failed!");
        return false;
    }
    goal_reached_ = false;
    // 0.获取轨迹终点的位姿
    geometry_msgs::PoseStamped goal_pose;
    const geometry_msgs::PoseStamped& plan_goal_pose = global_plan_.back();
    try {
        geometry_msgs::TransformStamped transform =
            tf_->lookupTransform(global_frame_, ros::Time(), plan_goal_pose.header.frame_id,
                                 plan_goal_pose.header.stamp, plan_goal_pose.header.frame_id, ros::Duration(0.5));

        tf2::doTransform(plan_goal_pose, goal_pose, transform);
    } catch (tf2::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    } catch (tf2::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    } catch (tf2::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        if (global_plan_.size() > 0)
            ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame_.c_str(),
                      (unsigned int)global_plan_.size(), global_plan_[0].header.frame_id.c_str());

        return false;
    }
    // 1.获取机器人当前位姿
    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_ERROR("[m100_dock_local_planner]: Could not get robot pose");
        return false;
    }
    // 2.获得目标点是处于机器人前方还是后方的方向，目标点在前方，则为正，后方为负.若机器人朝向与目标点朝向完全平行，则为零。即到位
    int direction;
    direction = getDirection(robot_pose, goal_pose);
    // 3.计算是否已到达阈值或是震荡
    double dis;
    dis = getDistance(robot_pose, goal_pose);
    dis *= direction;
    if (fabs(dis) < tolerance_) {
        goal_reached_    = true;
        cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
        ROS_INFO("[m100_dock_local_planner]: Reached! \n robot pose = (%f, %f, %f), goal pose = (%f, %f, "
                 "%f), dist = %f \n",
                 robot_pose.pose.position.x, robot_pose.pose.position.y, tf::getYaw(robot_pose.pose.orientation),
                 goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation),
                 fabs(dis));
        return true;
    } else if (dis * dis_old_ < 0.000000000 || dis == 0.00000000) {
        goal_reached_    = true;
        cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
        ROS_WARN("[m100_dock_local_planner]: Oscillation! and stop. \n robot pose = (%f, %f, %f), goal pose = (%f, %f, "
                 "%f), dist = %f \n",
                 robot_pose.pose.position.x, robot_pose.pose.position.y, tf::getYaw(robot_pose.pose.orientation),
                 goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation),
                 fabs(dis));
        return true;
    }
    dis_old_ = dis;

    // 4.计算返回充电or结束充电的速度，由比例控制完成，当为返回充电时，速度为负，绝对值逐渐减小；当为结束充电时，速度为正，逐渐增大
    if (dis > 0)
        cmd_vel.linear.x = clipGoHead(kp_ * dis, max_vel_, min_vel_);  //结束充电
    else if (dis < 0)
        cmd_vel.linear.x = clipGoBack(kp_ * dis, max_vel_, min_vel_);  //返回充电
    else
        cmd_vel.linear.x = 0.0;
    if (direction == 1)
        ROS_INFO_THROTTLE(1,
                          "[dock_local_planner]: Now the robot is leaving the charging pile at a speed of  %f m/s \n "
                          "robot pose = (%f, %f, %f), goal pose = (%f, %f, %f), dist = %f",
                          cmd_vel.linear.x, robot_pose.pose.position.x, robot_pose.pose.position.y,
                          tf::getYaw(robot_pose.pose.orientation), goal_pose.pose.position.x, goal_pose.pose.position.y,
                          tf::getYaw(goal_pose.pose.orientation), fabs(dis));
    else if (direction == -1)
        ROS_INFO_THROTTLE(1,
                          "[dock_local_planner]: Now the robot is approaching the charging pile at a speed of  %f m/s "
                          "\n robot pose = (%f, %f, %f), goal pose = (%f, %f, %f), dist = %f",
                          cmd_vel.linear.x, robot_pose.pose.position.x, robot_pose.pose.position.y,
                          tf::getYaw(robot_pose.pose.orientation), goal_pose.pose.position.x, goal_pose.pose.position.y,
                          tf::getYaw(goal_pose.pose.orientation), fabs(dis));
    return true;
}

bool M100DockLocalPlannerROS::isGoalReached() {
    if (!isInitialized()) {
        ROS_ERROR("[m100_dock_local_planner]: isGoalReached init failed!");
        return false;
    }
    if (goal_reached_) {
        reSetState();
        ROS_INFO("[m100_dock_local_planner]: Dock navigation over");
        return true;
    }
    return false;
}

void M100DockLocalPlannerROS::reSetState() {
    dis_old_      = 0.0;
    goal_reached_ = false;
}

double M100DockLocalPlannerROS::getDistance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2) {
    return sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2));
}

int M100DockLocalPlannerROS::getDirection(geometry_msgs::PoseStamped robot_pose, geometry_msgs::PoseStamped goal_pose) {
    // 0.获取robot pose在全局坐标系下的位姿
    tf::Transform tf_pose_to_map;
    tf::Vector3 pose_to_map;
    tf::Quaternion quaternion;
    quaternion.setX(robot_pose.pose.orientation.x);
    quaternion.setY(robot_pose.pose.orientation.y);
    quaternion.setZ(robot_pose.pose.orientation.z);
    quaternion.setW(robot_pose.pose.orientation.w);
    pose_to_map = {robot_pose.pose.position.x, robot_pose.pose.position.y, 0.0};
    tf_pose_to_map.setOrigin(pose_to_map);
    tf_pose_to_map.setRotation(quaternion);

    // 1.获取目标点在全局坐标系下的位置
    tf::Vector3 goal_to_map;
    goal_to_map = {goal_pose.pose.position.x, goal_pose.pose.position.y, 0.0};

    // 2.获取目标点在robot pose坐标系下的位置
    tf::Vector3 goal_to_robot_pose;
    goal_to_robot_pose = tf_pose_to_map.inverse() * goal_to_map;

    // 3.输出目标点在机器人的前方or后方
    if (goal_to_robot_pose.x() != 0.00000000000000)
        return goal_to_robot_pose.x() / fabs(goal_to_robot_pose.x());
    else
        return 0;
}

double M100DockLocalPlannerROS::clipGoHead(double v, double v_max, double v_min) {
    v = v_max - v;
    if (v <= v_min)
        v = v_min;
    return v;
}

double M100DockLocalPlannerROS::clipGoBack(double v, double v_max, double v_min) {
    int i = sign(v);
    if (fabs(v) > v_max)
        return v_max * i;
    else if (fabs(v) < v_min)
        return v_min * i;
    return v;
}

int M100DockLocalPlannerROS::sign(double x) {
    if (x > 0)
        return 1;
    else if (x == 0)
        return 0;
    else
        return -1;
}

}
