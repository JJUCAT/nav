//
// Created by czh on 8/18/21.
//

#include <nem_local_planner/pid_control.h>

namespace pid_control {
PidControl::PidControl() : error_last_(0.0), integral_(0.0) {
    ros::NodeHandle nh;
    reset_pub_ = nh.advertise<std_msgs::Float64>("pid_reset", 1);
    head_pose_pub_ = nh.advertise<visualization_msgs::Marker>("head_pose", 1);
}

PidControl::~PidControl() {}

float PidControl::getPidControl(double Kp, double Ki, double Kd, double max) {
    float result;
    float del_pid_time = 0.05;
    integral_ += error_last_ * del_pid_time;
    if (error_last_ != 0.0)
        result = Kp * error_ + Ki * integral_ + Kd * (error_ - error_last_) / del_pid_time;
    else
        result = Kp * error_ + Ki * integral_;
    error_last_ = error_;

    if (fabs(result) > max)
        result = (fabs(result) / result) * max;

    return result;
}

void PidControl::reSet(const std::string &cmd) {
    integral_   = 0.0;
    error_last_ = 0.0;
    std_msgs::Float64 msg;
    if(cmd == "reset")
        msg.data = 1.0;
    else if(cmd == "keep_plot")
        msg.data = 2.0;
    reset_pub_.publish(msg);
}

void PidControl::setPathAndPose(const std::vector<geometry_msgs::PoseStamped> &path, const geometry_msgs::PoseStamped &pose, const double &vx, const double &left_dist){
    double min_dist_sq = std::numeric_limits<double>::max();
    int min_idx = -1;
    for (int i = 0; i < path.size(); i++)
    {
        double dist_sq = sqrt(pow(path[i].pose.position.x - pose.pose.position.x, 2) + pow(path[i].pose.position.y - pose.pose.position.y, 2));
        if (dist_sq < min_dist_sq)
        {
            min_dist_sq = dist_sq;
            min_idx = i;
        }
    }
    geometry_msgs::PoseStamped head_pose;
    head_pose.pose.position.x = path[min_idx].pose.position.x;
    head_pose.pose.position.y = path[min_idx].pose.position.y;
    angle_diff_ = getAngleDiff(pose, path.back());
    head_dist_ = sqrt(pow(head_pose.pose.position.x - pose.pose.position.x, 2) + pow(head_pose.pose.position.y - pose.pose.position.y, 2));
    offset_dist_ = getOffsetDist(pose, path.front(), path.back());
    //error_ = purePursitFunctionCost(head_dist_, angle_diff_ * vx / fabs(vx));
    double tol_dist = sqrt(pow(path.front().pose.position.x - path.back().pose.position.x, 2) + pow(path.front().pose.position.y - path.back().pose.position.y, 2));
    error_ = stanleyFunctionCost(offset_dist_, vx, angle_diff_, 0.45, left_dist / tol_dist, left_dist);
    headPoseMarkerPub(head_pose);
}

double PidControl::getAngleDiff(const geometry_msgs::PoseStamped &pose, const geometry_msgs::PoseStamped &target) {
    //判断当前点P的朝向在有向线段PB的坐标系上夹角为正或负，B为路径终点
    tf::Transform transform_P_to_map;
    tf::Vector3 P_to_map;
    P_to_map = {pose.pose.position.x, pose.pose.position.y, 0.0};
    transform_P_to_map.setOrigin(P_to_map);
    tf::Quaternion rotation_P_to_map;
    rotation_P_to_map.setX(pose.pose.orientation.x);
    rotation_P_to_map.setY(pose.pose.orientation.y);
    rotation_P_to_map.setZ(pose.pose.orientation.z);
    rotation_P_to_map.setW(pose.pose.orientation.w);
    transform_P_to_map.setRotation(rotation_P_to_map);

    tf::Transform transform_PB_to_map;
    tf::Vector3 PB_to_map;
    PB_to_map = {pose.pose.position.x, pose.pose.position.y, 0.0};
    transform_PB_to_map.setOrigin(PB_to_map);
    transform_PB_to_map.setRotation(tf::createQuaternionFromYaw(atan2(target.pose.position.y - pose.pose.position.y, target.pose.position.x - pose.pose.position.x)));

    tf::Transform transform_P_to_PB;
    transform_P_to_PB = transform_PB_to_map.inverse() * transform_P_to_map;

    double angle_diff = tf::getYaw(transform_P_to_PB.getRotation());
    return -angle_diff;

}

double PidControl::getOffsetDist(const geometry_msgs::PoseStamped &P, const geometry_msgs::PoseStamped &A, const geometry_msgs::PoseStamped &B) {
    double dis;
    double a, b, c;                 //以双精度保存变量
    a = (P.pose.position.x - A.pose.position.x) * (B.pose.position.y - A.pose.position.y);  //分子的左半部分
    b = (P.pose.position.y - A.pose.position.y) * (A.pose.position.x - B.pose.position.x);  //分子的右半部分
    c = a + b;                      //二者相加
    c *= c;                         //平方(pow(c,2)貌似在这里更加麻烦)
    a = pow(B.pose.position.y - A.pose.position.y, 2);          //分母左半部分
    b = pow(A.pose.position.x - B.pose.position.x, 2);          //分母右半部分
    if (a + b == 0.000000000) {
        dis = 0.0;
        return dis;
    }
    c /= (a + b);               //分子分母相除
    dis = sqrt(c);              //开方
    if (translationP(A, B, P))  // TranslationP true为顺时针，false为逆时针
        dis = -dis;
    return dis;
}

bool PidControl::translationP(const geometry_msgs::PoseStamped &A, const geometry_msgs::PoseStamped &B, const geometry_msgs::PoseStamped &P) {
    //判断P点处于有向线段A->B的位置，在順時針位置則爲true
    tf::Transform transform_A_to_map;
    tf::Vector3 origin_A_to_map;
    origin_A_to_map = {A.pose.position.x, A.pose.position.y, 0.0};

    double yaw_AB_to_map;
    yaw_AB_to_map = atan2(B.pose.position.y - A.pose.position.y, B.pose.position.x - A.pose.position.x);

    transform_A_to_map.setOrigin(origin_A_to_map);
    transform_A_to_map.setRotation(tf::createQuaternionFromYaw(yaw_AB_to_map));

    tf::Vector3 origin_P_to_map;
    origin_P_to_map = {P.pose.position.x, P.pose.position.y, 0.0};

    tf::Vector3 P_to_A = transform_A_to_map.inverse() * origin_P_to_map;

    if (P_to_A.y() > 0.000)
        return true;
    else
        return false;
}

double PidControl::purePursitFunctionCost(double distance, double theta) { return distance * sin(theta); }

double PidControl::stanleyFunctionCost(double d, double v, double theta, double k, double dist_k, double left_dis) {
    double robot_r;
    double angle_diff = theta;
    if(fabs(theta) >= 3.1415926 / 2)
        angle_diff = (3.1415926 - fabs(theta)) * theta / fabs(theta);
        robot_r =  sin(atan2(k * d, fabs(v)) + angle_diff * v / fabs(v));
        // 避免Stanley原理导致的低速度、靠近目标点时的角度调整过大
        if(left_dis < 0.15)
            robot_r =  sin(atan2(k * d, fabs(v)) + angle_diff * v / fabs(v)) * dist_k;
    return robot_r;

}

void PidControl::headPoseMarkerPub(const geometry_msgs::PoseStamped &head_pose){
    visualization_msgs::Marker head_pose_vis;

    head_pose_vis.header.frame_id = "map";
    head_pose_vis.header.stamp = ros::Time();
    head_pose_vis.ns = "head_pose";
    head_pose_vis.id = 1;
    head_pose_vis.type = visualization_msgs::Marker::SPHERE;
    head_pose_vis.action = visualization_msgs::Marker::ADD;
    head_pose_vis.pose.position.x = head_pose.pose.position.x;
    head_pose_vis.pose.position.y = head_pose.pose.position.y;
    head_pose_vis.pose.position.z = 0.0;
    head_pose_vis.scale.x = 0.15;
    head_pose_vis.scale.y = 0.15;
    head_pose_vis.scale.z = 0.15;
    head_pose_vis.color.a = 1.0; // Don't forget to set the alpha!
    head_pose_vis.color.r = 1.0;
    head_pose_vis.color.g = 0.0;
    head_pose_vis.color.b = 0.0;
    head_pose_pub_.publish(head_pose_vis);
}

}  // namespace pid_control
