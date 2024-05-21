#include <path_follower/path_follow_control.h>
namespace control {

void ControllerPathFollow::initialize(const std::string name, tf2_ros::Buffer* tf, navit_costmap::Costmap2DROS* costmap_ros)
{
// check if the plugin is already initialized
    if(!initialized_) {
        //initial tf handle thread.
        costmap_ros_ptr_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        // create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle nh("~/" + name);
        path_follow_cfg_.robot_frame = costmap_ros->getGlobalFrameID();
        tf_buffer_ = tf;
        astar_search_.init(name);
        path_spline_ptr_->initialize(name);
        nh.param<std::string>("robot/global_frame", path_follow_cfg_.global_frame, "map");
        nh.param<std::string>("robot/robot_frame", path_follow_cfg_.robot_frame, "base_link");
        nh.param("robot/extend_path_dis", path_follow_cfg_.extend_path_dis, 0.0);
        // nh.param("robot/padding_footprint", path_follow_cfg_.padding_footprint, 0.0);

        nh.param("robot/use_avoidance", path_follow_cfg_.use_avoidance, true);
        nh.param("robot/diff/max_vel_x", path_follow_cfg_.diff_cfg.max_vel_x, 0.4);
        nh.param("robot/diff/min_vel_x", path_follow_cfg_.diff_cfg.min_vel_x, 0.1);

        if (path_follow_cfg_.diff_cfg.min_vel_x >= 0.2) {
            ROS_WARN("The minimum velocity in the global frame should be less than 0.2.");
            path_follow_cfg_.diff_cfg.min_vel_x = 0.2;
        }
        ROS_INFO("Min vel x: %f, Max vel x: %f" , path_follow_cfg_.diff_cfg.min_vel_x, path_follow_cfg_.diff_cfg.max_vel_x);


        nh.param("robot/global_frame/type/diff/max_vel_x_backwards", path_follow_cfg_.diff_cfg.max_vel_x_backwards, 0.4);
        nh.param("robot/global_frame/type/diff/max_vel_theta", path_follow_cfg_.diff_cfg.max_vel_theta, 0.4);
        nh.param("robot/global_frame/type/diff/acc_lim_x", path_follow_cfg_.diff_cfg.acc_lim_x, 0.4);
        nh.param("robot/global_frame/type/diff/dec_lim_x", path_follow_cfg_.diff_cfg.dec_lim_x, 0.4);
        nh.param("robot/global_frame/type/diff/acc_lim_theta", path_follow_cfg_.diff_cfg.acc_lim_theta, 0.4);

        nh.param("robot/global_frame/type/diff/use_angular_vel", path_follow_cfg_.simple_car_cfg.use_angular_vel, false);
        nh.param("robot/global_frame/type/diff/wheelbase", path_follow_cfg_.simple_car_cfg.wheelbase, 0.4);
        nh.param("robot/global_frame/type/diff/max_vel_x", path_follow_cfg_.simple_car_cfg.max_vel_x, 0.8);
        nh.param("robot/global_frame/type/diff/max_vel_x_backwards", path_follow_cfg_.simple_car_cfg.max_vel_x_backwards, 0.4);
        nh.param("robot/global_frame/type/diff/max_steering_angle", path_follow_cfg_.simple_car_cfg.max_steering_angle, 0.4);
        nh.param("robot/global_frame/type/diff/acc_lim_x", path_follow_cfg_.simple_car_cfg.acc_lim_x, 0.4);
        nh.param("robot/global_frame/type/diff/dec_lim_x", path_follow_cfg_.simple_car_cfg.dec_lim_x, 0.4);
        nh.param("robot/global_frame/type/diff/max_steering_rate", path_follow_cfg_.simple_car_cfg.max_steering_rate, 0.4);

        nh.param("robot/dis_tolerance", path_follow_cfg_.dis_tolerance, 1.0);
        nh.param("robot/theta_tolerance", path_follow_cfg_.theta_tolerance, 1.0);
        nh.param("robot/move_to_pose", path_follow_cfg_.avoidance_cfg.move_to_pose, false);
        nh.param("robot/look_path_ahead", path_follow_cfg_.look_path_ahead, 20000);
        nh.param("robot/min_look_ahead_index", path_follow_cfg_.min_look_ahead_dis, 10.0f);
        nh.param("robot/look_ahea_dis_ratio", path_follow_cfg_.look_ahea_dis_ratio, 50.0f);

        nh.param("robot/robot_box/front_left_x",  path_follow_cfg_.robot_box.front_left_x, 1.0);
        nh.param("robot/robot_box/front_left_y",  path_follow_cfg_.robot_box.front_left_y, 1.0);
        nh.param("robot/robot_box/front_right_x", path_follow_cfg_.robot_box.front_right_x, 1.0);
        nh.param("robot/robot_box/front_right_y", path_follow_cfg_.robot_box.front_right_y, -1.0);
        nh.param("robot/robot_box/rear_right_x",  path_follow_cfg_.robot_box.rear_right_x, -1.0);
        nh.param("robot/robot_box/rear_right_y",  path_follow_cfg_.robot_box.rear_right_y, -1.0);
        nh.param("robot/robot_box/rear_left_x",   path_follow_cfg_.robot_box.rear_left_x, -1.0);
        nh.param("robot/robot_box/rear_left_y",   path_follow_cfg_.robot_box.rear_left_y, 1.0);

        //avoidance cfg
        nh.param("avoidance/collision_check_box_sizes", path_follow_cfg_.avoidance_cfg.collision_check_box_sizes, 5);
        nh.param("avoidance/box_interval", path_follow_cfg_.avoidance_cfg.box_interval, 40);

        nh.param("avoidance/use_clear_costmap", path_follow_cfg_.avoidance_cfg.use_clear_costmap, false);
        nh.param("avoidance/clear_costmap_wait_time", path_follow_cfg_.avoidance_cfg.clear_costmap_wait_time, 5.0);
        nh.param("avoidance/use_recovery", path_follow_cfg_.avoidance_cfg.use_recovery, false);
        nh.param("avoidance/use_eliminate_shocks", path_follow_cfg_.avoidance_cfg.use_eliminate_shocks, false);
        // nh.param("avoidance/dt", path_follow_cfg_.avoidance_cfg.dt, 2.0f);
        // nh.param("avoidance/min_step_distance", path_follow_cfg_.avoidance_cfg.min_step_distance, 0.05f);
        // nh.param("avoidance/forward_distance", path_follow_cfg_.avoidance_cfg.forward_distance, 2.0f);


        ROS_INFO("robot/look_path_ahead is %d", path_follow_cfg_.look_path_ahead);
        ROS_INFO("robot/min_look_ahead_index is %f", path_follow_cfg_.min_look_ahead_dis);
        ROS_INFO("robot/look_ahea_dis_ratio is %f", path_follow_cfg_.look_ahea_dis_ratio);

        if (path_follow_cfg_.avoidance_cfg.collision_check_box_sizes < 3 ) {
            path_follow_cfg_.avoidance_cfg.collision_check_box_sizes = 3;
            ROS_WARN("Avoidance param collision check box size is less 3, set 3.");
        }

        debug_pub_ = nh.advertise<geometry_msgs::PoseArray>("debug_pose", 1);
        clear_costmap_srv_ = nh.serviceClient<std_srvs::Empty>("/clear_controller_costmap");
        collision_check_poses_pub_ = nh.advertise<geometry_msgs::PoseArray>("collision_check_poses", 1);
        footprint_marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("footprint_marker_array", 1);
        ompl_path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
        split_jount_path_pub_ = nh.advertise<nav_msgs::Path>("split_jount_path", 1);

        smac_planner_.initialize(nh.getNamespace(), costmap_ros_);

        tf_ = tf;
        tf_handle_thread_ptr_ = new boost::thread(boost::bind(&ControllerPathFollow::tfHandle, this));
        //initial path follow replan
        replan_thread_ptr_ = new boost::thread(boost::bind(&ControllerPathFollow::replanHandle, this));
        // collision_checker_.reset(new navit_collision_checker::FootprintCollisionChecker< navit_costmap::Costmap2D*>(costmap_));
        // ompl_ros_ptr_ = std::make_shared<OmplRos>(nh, tf_buffer_, costmap_ros);
        initialized_ = true;
        ROS_INFO("Path follower controller initialized.");
    } else {
        ROS_WARN("Path follower has already been initialized, do nothing.");
    }
}

void ControllerPathFollow::setZeroControlCommand(geometry_msgs::Twist& cmd_vel) {
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
}

void ControllerPathFollow::setZeroControlComannd(proto::ControlCommand* control_command) {
    control_command->set_speed(0.0);
    control_command->set_front_wheel_angle(0.0);
    control_command->set_right_wheel_speed(0.0);
    control_command->set_left_wheel_speed(0.0);
}

geometry2::Vec3d ControllerPathFollow::transOrientation2euler(const geometry_msgs::Quaternion orientation) {
    geometry2::Vec3d euler;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(orientation, quat);
    tf::Matrix3x3(quat).getRPY(euler.x, euler.y, euler.z);
    return euler;
}

geometry_msgs::Twist ControllerPathFollow::computeVelocityCommands(const geometry_msgs::PoseStamped& current_pose,
                                                                   const geometry_msgs::Twist& current_vel) {
    geometry_msgs::Twist cmd_vel;
    proto::ControlCommand control_command;
    proto::ControlCommand* control_command_ptr = &control_command;
    isCollisionImminent();
    if ((goal_reached_ && collisioned_ && !replan_success_) || !tf_received_) {
        setZeroControlCommand(cmd_vel);
        return cmd_vel;
    }

    // if (replan_attemped_failed_) {
    //     setZeroControlCommand(cmd_vel);
    //     failed_flag_ = true;
    //     cmd_vel.linear.z = KBlockFlag_;
    //     return cmd_vel;
    // } else if (failed_flag_ && !replan_attemped_failed_) {
    //     setZeroControlCommand(cmd_vel);
    //     failed_flag_ = false;
    //     return cmd_vel;
    // }
    if (smooth_global_path_.path_points_size() == 0) {
        ROS_ERROR("Smooth global path size is 0.");
        setZeroControlCommand(cmd_vel);
    } else {
        switch (control_mode_) {
            case ROTATE_FOR_PRELINE:
                setControlAngle(anchor_point_yaw_, robot_state_, control_command);
                break;
            case SLOW_DOWN:
                setControl(slow_down_path_, robot_state_, distance_, control_command_ptr, false);
                break;
            case ROTATE_FOR_LINE:
                setControlAngle(target_yaw_, robot_state_, control_command);
                break;
            case FOLLOW_PATH:
                if (!setControl(smooth_global_path_, robot_state_, distance_, control_command_ptr, false)) {
                    cmd_vel.linear.z = KBlockFlag_;
                } else {
                    cmd_vel.linear.z = 0.0;
                }
                break;
            case ROTATE_FINIAL:
                setControlAngle((transOrientation2euler(final_pose_.pose.orientation)).z, robot_state_, control_command);
                break;
            default:
                break;
        }

        cmd_vel.linear.x = control_command.speed();
        cmd_vel.linear.y = 0;
        if (cmd_vel.linear.x > path_follow_cfg_.diff_cfg.max_vel_x) {
            cmd_vel.linear.x = path_follow_cfg_.diff_cfg.max_vel_x;
        }

        cmd_vel.linear.x = std::max(cmd_vel.linear.x, path_follow_cfg_.diff_cfg.min_vel_x);

        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = control_command.front_wheel_angle();
        linear_vel_ = cmd_vel.linear.x;
    }
    return cmd_vel;
}

void ControllerPathFollow::replanHandle(){
    ros::Rate rate(1);
    // while(1) {
    //     rate.sleep();
    // }
    astar_search_.setTolerance(5);
    while(ros::ok() && path_follow_cfg_.use_avoidance && replanHandle_done_) {
        if (replan_flag_ && !replan_success_) {
            init_replan_goal_ = replan_goal_;
            // ompl_ros_ptr_->planPath(replan_start_, replan_goal_, path_poses, true, false);
            nav_msgs::Path ompl_local_path;
            geometry_msgs::PoseStamped start_pose_stamped, goal_pose_stamped;
            start_pose_stamped.header.frame_id = path_follow_cfg_.global_frame;
            start_pose_stamped.pose = replan_start_;
            goal_pose_stamped.header.frame_id = path_follow_cfg_.global_frame;
            goal_pose_stamped.pose = replan_goal_;

            ompl_local_path = smac_planner_.makePlan(start_pose_stamped, goal_pose_stamped);
            ompl_local_path.header.frame_id = path_follow_cfg_.global_frame;
            ompl_path_pub_.publish(ompl_local_path);

            if (ompl_local_path.poses.size() == 0){
                ROS_ERROR("Cannot make plan.");
                replan_count_ --;
                if(replan_count_ <= 0) {
                    ROS_ERROR("Replan failed. We will try to recovery behavior.");
                    replan_attemped_failed_ = true;
                }
                robot_blocked_ = true;
                replan_success_ = false;
            } else {
                ROS_INFO("Make plan successfully.");
                replan_count_ = 5;
                replan_attemped_failed_ = false;
                robot_blocked_ = false;
                replan_success_ = true;
                control_mode_ = FOLLOW_PATH;
            }
            astar_search_.publishPoseArray(debug_pub_, "map");
            nav_msgs::Path local_path_ros = astar_search_.getRosPath();
            trajectory_viewer_.visualizeLocalPath(local_path_ros, "map");

            proto::Polyline local_path;
            // 将ompl_local_path 转换为proto::Polyline

            for (int i = 0; i < ompl_local_path.poses.size(); ++i) {
                proto::Vector3f* path_point_ptr = local_path.add_points();
                path_point_ptr->set_x(ompl_local_path.poses[i].pose.position.x);
                path_point_ptr->set_y(ompl_local_path.poses[i].pose.position.y);
            }
            if (replan_success_) {
                setReplan(replan_current_index_, replan_goal_index_, local_path);
            }
        }
    rate.sleep();
  }
}

// echo base_link to map from tf tree.
void ControllerPathFollow::tfHandle() {
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  while (ros::ok() && replanHandle_done_){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(path_follow_cfg_.global_frame, "base_link",
                               ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    try{
      listener.lookupTransform(path_follow_cfg_.global_frame, plan_frame_,
                               ros::Time(0), global_to_target_transform_);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    tf_received_ = true;
    robot_state_.mutable_position()->set_x(transform.getOrigin().x());
    robot_state_.mutable_position()->set_y(transform.getOrigin().y());
    robot_state_.mutable_position()->set_z(transform.getOrigin().z());

    robot_state_.mutable_velocity_flu()->set_x(0.0);
    robot_state_.mutable_velocity_flu()->set_y(0.0);
    robot_state_.mutable_velocity_flu()->set_z(0.0);

    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    robot_state_.mutable_roll_pitch_yaw()->set_x(roll);
    robot_state_.mutable_roll_pitch_yaw()->set_y(pitch);
    robot_state_.mutable_roll_pitch_yaw()->set_z(yaw);
    geometry_msgs::Quaternion qua = tf::createQuaternionMsgFromYaw(yaw);
    robot_state_.mutable_quaternion()->set_x(qua.x);
    robot_state_.mutable_quaternion()->set_y(qua.y);
    robot_state_.mutable_quaternion()->set_z(qua.z);
    robot_state_.mutable_quaternion()->set_w(qua.w);
    current_pose_ = geometry2::Vec2f(robot_state_.position().x(),robot_state_.position().y());
    current_pose_stamped_.header.frame_id = path_follow_cfg_.global_frame;
    current_pose_stamped_.pose.position.x = robot_state_.position().x();
    current_pose_stamped_.pose.position.y = robot_state_.position().y();
    current_pose_stamped_.pose.position.z = robot_state_.position().z();

    current_pose_stamped_.pose.orientation.x = qua.x;
    current_pose_stamped_.pose.orientation.y = qua.y;
    current_pose_stamped_.pose.orientation.z = qua.z;
    current_pose_stamped_.pose.orientation.w = qua.w;
    rate.sleep();
  }
}

//TODO: A formal parameter needs to be added here to distinguish different paths.
bool ControllerPathFollow::setPlan(const nav_msgs::Path& plan) {

    std::lock_guard<std::mutex> lock(global_variable_mutex_);
    //reset status if new plan reached.
    robot_blocked_ = false;
    first_hit_ = true;
    set_control_first_hit_ = true;
    smooth_global_path_.clear_path_points();
    path_point_vec_.clear();
    path_point_list_.clear();
    goal_reached_ = false;
    control_mode_ = FOLLOW_PATH;

    nav_msgs::Path temp_plan;
    plan_frame_ = plan.header.frame_id;
    if (plan.poses.size() < 4) {
        ROS_ERROR("Origin Path points is too less. poses size is %zu", plan.poses.size());
        goal_reached_ = true;
    }

    temp_plan = plan;
    //temp_plan = resamplePath(plan, 0.01);

    geometry2::Vec2f last_pose;
    geometry2::Vec2f next_to_last_pose;

    last_pose.x = temp_plan.poses[temp_plan.poses.size() - 1].pose.position.x;
    last_pose.y = temp_plan.poses[temp_plan.poses.size() - 1].pose.position.y;

    next_to_last_pose.x = temp_plan.poses[temp_plan.poses.size() - 2].pose.position.x;
    next_to_last_pose.y = temp_plan.poses[temp_plan.poses.size() - 2].pose.position.y;

    float path_tail_yaw = (last_pose - next_to_last_pose).heading();
    final_pose_ = temp_plan.poses.back();

    // we expand path a little
    if (first_hit_) {
        geometry_msgs::PoseStamped expand_pose;
        first_hit_ = false;
        expand_pose.pose.position.x = last_pose.x + path_follow_cfg_.extend_path_dis * cos(path_tail_yaw);
        expand_pose.pose.position.y = last_pose.y + path_follow_cfg_.extend_path_dis * sin(path_tail_yaw);
        temp_plan.poses.push_back(expand_pose);
    }

    // TODO: If smoothed path failed, try to use origin path
    if (path_follow_cfg_.avoidance_cfg.move_to_pose == true) {
        path_spline_ptr_->setStep(10);
    } else {
        path_spline_ptr_->setStep(1);
    }

    if (path_spline_ptr_->pathSmoother(temp_plan, smooth_global_path_)) {
        trajectory_viewer_.visualizePreSmoothPath(smooth_global_path_, path_follow_cfg_.global_frame);
        coordinate_convert_ptr_->clear();
        coordinate_convert_ptr_->setPath(smooth_global_path_);
        proto::PathPoint path_point, *path_point_ptr;
        for (int i = 0; i < smooth_global_path_.path_points().size(); ++i) {
            path_point = smooth_global_path_.path_points(smooth_global_path_.path_points().size() - i -1);
            path_point_vec_.push_back(path_point);

            path_point_ptr = init_global_path_.add_path_points();
            path_point_ptr->CopyFrom(smooth_global_path_.path_points(i));
            path_point_list_.push_back(path_point);
        }

        geometry2::Vec2f initial_pose_1;
        geometry2::Vec2f initial_pose_2;
        PathPointIter iter;

        if (path_matcher_ptr_->findInCiclePoint(smooth_global_path_, geometry2::Vec2f(robot_state_.position()), &iter, 0.05, 0)) {
            initial_pose_1.x = iter->position().x();
            initial_pose_1.y = iter->position().y();

            initial_pose_2.x = (iter + 1)->position().x();
            initial_pose_2.y = (iter + 1)->position().y();

            target_yaw_ = (initial_pose_2 - initial_pose_1).heading();
        } else {
        }

        return true;
    } else {
        ROS_ERROR("Smooth path failed, using origon path.");
        return false;
    }
}
// set replan , TODO: rescontruct a split joint bt node later
bool ControllerPathFollow::setReplan(const int conncet_begin, const int connect_end,
                                     const proto::Polyline& local_path) {
    if (local_path.points().size() < 2) {
        return false;
    }
    std::vector<proto::Vector3f> replan_poses;
    for (int i = 0; i < local_path.points().size(); ++i) {
        replan_poses.push_back(local_path.points(i));
    }

    for (int i = connect_end + 4; i < path_point_vec_.size(); ++i) {
        replan_poses.push_back(path_point_vec_[path_point_vec_.size() - i -1].position());
    }

    nav_msgs::Path split_joint_path;
    geometry_msgs::PoseStamped pose;
    split_joint_path.header.frame_id = path_follow_cfg_.global_frame;
    for (uint i = 0; i < replan_poses.size(); ++i) {
        pose.header.frame_id = path_follow_cfg_.global_frame;
        pose.pose.position.x = replan_poses[i].x();
        pose.pose.position.y = replan_poses[i].y();
        pose.pose.orientation.w = 1.0;
        split_joint_path.poses.push_back(pose);
    }

    split_jount_path_pub_.publish(split_joint_path);
    // geometry2::Vec2f last_pose;
    // geometry2::Vec2f next_to_last_pose;

    // last_pose.x = split_joint_path.poses[split_joint_path.poses.size() - 1].pose.position.x;
    // last_pose.y = split_joint_path.poses[split_joint_path.poses.size() - 1].pose.position.y;

    // // path is more straight if select the num 10
    // next_to_last_pose.x = split_joint_path.poses[split_joint_path.poses.size() - 2].pose.position.x;
    // next_to_last_pose.y = split_joint_path.poses[split_joint_path.poses.size() - 2].pose.position.y;

    // float path_tail_yaw = (last_pose - next_to_last_pose).heading();

    // we expand path a little
    // if (first_hit_) {
    //     geometry_msgs::PoseStamped expand_pose;
    //     first_hit_ = false;
    //     expand_pose.pose.position.x = last_pose.x + 0.3f * cos(path_tail_yaw);
    //     expand_pose.pose.position.y = last_pose.y + 0.3f * sin(path_tail_yaw);
    //     split_joint_path.poses.push_back(expand_pose);
    // }

    proto::Path smooth_path;
    if (path_follow_cfg_.avoidance_cfg.move_to_pose == true) {
        path_spline_ptr_->setStep(4);
    } else {
        path_spline_ptr_->setStep(1);
    }

    smooth_global_path_.clear_path_points();
    if (!path_spline_ptr_->pathSmoother(split_joint_path, smooth_global_path_)) {
        ROS_ERROR("Path smooth failed.");
    }
    trajectory_viewer_.visualizeLocalSmoothPath(smooth_global_path_, path_follow_cfg_.global_frame);

    coordinate_convert_ptr_->clear();


    coordinate_convert_ptr_->setPath(smooth_global_path_);
    //delta yaw can be considered to judge if rotate or not.
    control_mode_ = FOLLOW_PATH;


    // geometry2::Vec2f initial_pose_1;
    // initial_pose_1.x = smooth_global_path_.path_points(0).position().x();
    // initial_pose_1.y = smooth_global_path_.path_points(0).position().y();

    // geometry2::Vec2f initial_pose_2;
    // initial_pose_2.x = smooth_global_path_.path_points(1).position().x();
    // initial_pose_2.y = smooth_global_path_.path_points(1).position().y();

    // target_yaw_ = (initial_pose_2 - initial_pose_1).heading();
    // 对path_point_vec_上锁
    std::lock_guard<std::mutex> lock(global_variable_mutex_);
    path_point_vec_.clear();
    for (int i = 0; i < smooth_global_path_.path_points().size(); ++i) {
        proto::PathPoint path_point;
        path_point = smooth_global_path_.path_points(smooth_global_path_.path_points().size() - i -1);
        path_point_vec_.push_back(path_point);
    }
    set_replan_ = true;
    return true;
}

bool ControllerPathFollow::setControl(const proto::Path& path, const proto::RobotState& robot_state, const double& target_distance,
                                      proto::ControlCommand* control_command, bool reverse) {
    std::lock_guard<std::mutex> lock(global_variable_mutex_);
    resetYawPid();
    Vec2f robot_sl;
    //cut path
    proto::Path path_proto;
    proto::PathPoint *proto_point;
    int path_size = path_follow_cfg_.look_path_ahead < path_point_vec_.size() - 1 ? path_follow_cfg_.look_path_ahead : path_point_vec_.size() - 1;
    for (int i = 0; i < path_size; ++i) {
        geometry_msgs::PoseStamped original_pose;
        original_pose.pose.position.x = path_point_vec_[path_point_vec_.size() - 1 - i].position().x();
        original_pose.pose.position.y = path_point_vec_[path_point_vec_.size() - 1 - i].position().y();
        original_pose.pose.position.z = path_point_vec_[path_point_vec_.size() - 1 - i].position().z();

        tf::Quaternion quat;
        quat.setRPY(0, 0, path_point_vec_[path_point_vec_.size() - 1 -i].theta());
        original_pose.pose.orientation.x = quat.x();
        original_pose.pose.orientation.y = quat.y();
        original_pose.pose.orientation.z = quat.z();
        original_pose.pose.orientation.w = quat.w();

        tf::Vector3 orig_position(
            original_pose.pose.position.x,
            original_pose.pose.position.y,
            original_pose.pose.position.z);

        tf::Vector3 transformed_position = global_to_target_transform_ * orig_position;

        tf::Quaternion orig_orientation(
            original_pose.pose.orientation.x,
            original_pose.pose.orientation.y,
            original_pose.pose.orientation.z,
            original_pose.pose.orientation.w);

        tf::Quaternion transformed_orientation = global_to_target_transform_ * orig_orientation;

        proto_point = path_proto.add_path_points();

        float x = transformed_position.x();
        float y = transformed_position.y();
        float z = transformed_position.z();

        proto_point->mutable_position()->set_x(x);
        proto_point->mutable_position()->set_y(y);
        proto_point->mutable_position()->set_z(z);

        geometry2::Vec3d euler;
        tf::Matrix3x3(transformed_orientation).getRPY(euler.x, euler.y, euler.z);

        proto_point->set_theta(euler.z);
        proto_point->set_speed(path_point_vec_[path_point_vec_.size() - 1 -i].speed());
    }
    proto::Path smooth_path;

    // path_spline_ptr_->pathSmoother(path_proto, smooth_path);
    // coordinate_convert_ptr_->clear();
    // coordinate_convert_ptr_->setPath(smooth_path);
    // coordinate_convert_ptr_->xyToSl(Vec2f(robot_state.position()), &robot_sl);

    if (path_proto.path_points().size() <= path.path_points().size()) {
        distance_ = path.path_points(path_proto.path_points().size() - 1).s() - robot_sl.x;
    }

    ROS_INFO_STREAM_THROTTLE(1.0, "Distance to goal: " << distance_);
    PathPointIter iter;
    int index = 0;
    float speed_cmd = 0.0;
    if (path_matcher_ptr_->findNearestPoint(path_proto, geometry2::Vec2f(robot_state.position()), &iter, &index)) {
        speed_cmd = (iter)->speed();
        //TODO(czk): param
        if (speed_cmd > last_speed_ && last_speed_ > 0) {
            speed_cmd = last_speed_ + (speed_cmd - last_speed_) * 0.05;
        } else if (last_speed_ > 0) {
            speed_cmd = last_speed_ + (speed_cmd - last_speed_) * 0.2;
        }
    } else {
        ROS_INFO("Cannot find speed.");
        return false;
    }

    for (int i = 2; i < index; i++) {
        if(path_point_vec_.size() > 0) {
            path_point_vec_.pop_back();
            if (path_point_list_.size() > 0) {
                path_point_list_.pop_front();
            }
         }
    }
    //Prune path
    replan_current_index_ = index;
    // smooth velocity reach goal soon.
    if (target_distance < 2.0) {
        speed_cmd = std::min(speed_cmd, std::max(float((target_distance * 0.4)), float(0.05)));
        speed_buffer_.push_back(speed_cmd);
        if (speed_buffer_.size() > 5) speed_buffer_.erase(speed_buffer_.begin());
        double total_speed = 0;
        for (auto speed : speed_buffer_) total_speed = total_speed + speed;
            speed_cmd = total_speed / speed_buffer_.size();
    } else {
        if (speed_buffer_.size() > 0) speed_buffer_.clear();
    }

    if (should_rotate_) {
        speed_cmd = round_corner_v_;
    }
    float distance2replangoal = 0.0f;
    float x = robot_state.position().x() - init_replan_goal_.position.x;
    float y = robot_state.position().y() - init_replan_goal_.position.y;
    distance2replangoal = sqrt(x * x + y * y);
    if (control_mode_ == FOLLOW_PATH  && distance_ < 0.06 || distance_ < 0.06) {
        // control_mode_ = ROTATE_FINIAL;
        goal_reached_ = true;
    }

    // just for purepursuit
    // init the feedback and feedforward curvature
    float fb_curvature = 0.0, fb_cmd = 0.0;
    int rotate_index = 0;
    float look_ahead_dist = previewDistSchedule(config_.fb_preview_point(), speed_cmd);
    // update matched points according to ff and fb preview distances
    geometry2::Vec2f anchor_p;
    proto::AnchorBox anchor_boxes;
    if (!path_matcher_ptr_->updateMatchedPoints(path_proto, robot_state, look_ahead_dist, reverse, anchor_p, &index,
                                                path_follow_cfg_.avoidance_cfg.collision_check_box_sizes,
                                                path_follow_cfg_.avoidance_cfg.box_interval, anchor_boxes, should_rotate_,
                                                rotate_index)) {
        ROS_ERROR("Cannot find match point.");
        return false;
    }

    trajectory_viewer_.visualizeAnchorPoses(anchor_boxes, path_follow_cfg_.global_frame);
    trajectory_viewer_.visualizePruneGlobalPath(path_proto, path_follow_cfg_.global_frame);
    trajectory_viewer_.visualizeAnchorPoint(anchor_p, path_follow_cfg_.global_frame);
    // update fb curvature
    PathMatcher::SteerError steer_error = path_matcher_ptr_->getSteerErr();
    float preview_mean_square = path_matcher_ptr_->getRobotTargetDistSquare(geometry2::Vec2f(robot_state.position()));
    fb_cmd = 2.0 * steer_error.lat / preview_mean_square;
    // calculate steering angle
    float steering_angle_cmd = atan((fb_cmd /* + ff_curvature*/) * 1.0);

    if (reverse) {
        steering_angle_cmd = -steering_angle_cmd;
    }
    //TODO: Add tolerance params
    if (target_distance < 0.1) {
        steering_angle_cmd = 0;
    }
    double angle_speed = clamp(std::tan(steering_angle_cmd) * speed_cmd / wheel_distance_, -1.9f, 1.9f);

    float angle_limit = std::atan(lateral_acc_limit_ * wheel_base_ / sqr(linear_vel_));

    steering_angle_cmd = clamp(steering_angle_cmd, -angle_limit, angle_limit);


    control_command->set_front_wheel_angle(steering_angle_cmd);
    speed_setpoint_ += clamp(speed_cmd - speed_setpoint_, -speed_slew_rate_, speed_slew_rate_);

    if (speed_cmd > 0) last_speed_ = speed_setpoint_; {
        control_command->set_speed(speed_setpoint_);
    }
    //Detect collision if donnot use avoidance, collision check only.
    //TODO: rescontruct a avoidance bt node later
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = robot_state_.position().x();
    pose.pose.position.y = robot_state_.position().y();
    pose.pose.position.z = robot_state_.position().z();

    pose.pose.orientation.x = robot_state_.quaternion().x();
    pose.pose.orientation.y = robot_state_.quaternion().y();
    pose.pose.orientation.z = robot_state_.quaternion().z();
    pose.pose.orientation.w = robot_state_.quaternion().w();

    if (path_follow_cfg_.use_avoidance) {
        static int collision_count = 0;
        if (isCollisionImminent(pose, control_command->speed(), angle_speed, anchor_boxes) == CUSHION) {
            ROS_WARN("Detected collision ahead, Slow down and look for broad road...");
            replan_flag_ = true;
            replan_success_ = false;
            collision_count ++;
            if (collision_count >= 5) {
                collision_count = 5;
            }
            if (control_mode_ != SLOW_DOWN && collision_count >= 5) {
                collision_count = 0;
                proto::Path slow_down_path;
                slow_down_path_.clear_path_points();
                //reserve path from current pose to next anchor pose
                for (unsigned int i = anchor_boxes.anchor_pose(0).index(); i < anchor_boxes.anchor_pose(1).index(); ++i) {
                    proto::PathPoint* path_point;
                    path_point = slow_down_path_.add_path_points();
                    path_point->CopyFrom(path_proto.path_points(i));
                }
                replan_current_index_ = anchor_boxes.anchor_pose(0).index();
                control_mode_ = SLOW_DOWN;
                return true;
            } else if (control_mode_ != SLOW_DOWN) {
            }
        } else if (isCollisionImminent(pose, control_command->speed(), angle_speed, anchor_boxes) == REPLAN) {
            setZeroControlComannd(control_command);
            replan_flag_ = true;
            replan_success_ = false;
            ROS_WARN_STREAM("Safe stop.");
            return true;
        } else if (isCollisionImminent(pose, control_command->speed(), angle_speed, anchor_boxes) == EMERGENCY_STOP) {
            replan_flag_ = false;
            setZeroControlComannd(control_command);
            ROS_WARN_STREAM("Emergency stop.");
            return true;
        } else {
            replan_flag_ = false;
            if (control_mode_ == SLOW_DOWN) {
                collision_count--;
                control_mode_ = FOLLOW_PATH;
                PathPointIter iter;
                int index;
                const geometry2::Vec2f current_pose = geometry2::Vec2f(robot_state.position());
                path_matcher_ptr_->findNearestPoint(path_proto, current_pose, &iter, &index);
            }
            collisioned_ = false;
            control_command->set_right_wheel_speed((2 * speed_setpoint_ + angle_speed * wheel_distance_) / 2);
            control_command->set_left_wheel_speed((2 * speed_setpoint_ - angle_speed * wheel_distance_) / 2);
        }
        if (target_distance < 0.1 && control_mode_ == SLOW_DOWN) {
            setZeroControlComannd(control_command);
        }
    } else {

    }
    return true;
}

bool ControllerPathFollow::setControlAngle(const double target_yaw, const proto::RobotState& robot_state,
                                           proto::ControlCommand& control_command) {
  double current_yaw = robot_state.roll_pitch_yaw().z();
  if (setControlAngle(target_yaw, current_yaw, control_command)) {
    return true;
  } else {
    return false;
  }
}

bool ControllerPathFollow::setControlAngle(const double target_yaw,
                                           const double current_yaw, proto::ControlCommand& control_command) {
    double anguler_velocity_cmd = 0.0;
    double error = current_yaw - target_yaw;
    error = normalizeAngle(error);

    config_rotation_control_.set_rotation_fb_gain(0.8);
    config_rotation_control_.set_max_rotation_speed(1.0);

    anguler_velocity_cmd = controller_yaw_pid_ptr_->setControl(0.0, error) * config_rotation_control_.rotation_fb_gain();
    anguler_velocity_cmd =
        clamp(anguler_velocity_cmd, -config_rotation_control_.max_rotation_speed(), config_rotation_control_.max_rotation_speed());
    if (anguler_velocity_cmd < 0 && anguler_velocity_cmd > -config_rotation_control_.rotation_threshold()) {
    anguler_velocity_cmd = -config_rotation_control_.rotation_threshold();
    }
  if (anguler_velocity_cmd > 0 && anguler_velocity_cmd < config_rotation_control_.rotation_threshold()) {
    anguler_velocity_cmd = config_rotation_control_.rotation_threshold();
  }
  steering_angle_setpoint_ += clamp(anguler_velocity_cmd - steering_angle_setpoint_, -angle_slew_rate_, angle_slew_rate_);
  // rotate error rad
  if (fabs(error) < 0.1) {
    control_command.set_front_wheel_angle(0.0);
    if (control_mode_ == ROTATE_FOR_PRELINE) {
      control_mode_ = SLOW_DOWN;
    } else if (control_mode_ == ROTATE_FOR_LINE) {
      control_mode_ = FOLLOW_PATH;
    } else if (control_mode_ == ROTATE_FINIAL) {
      goal_reached_ = true;
    }
  } else {
    control_command.set_front_wheel_angle(steering_angle_setpoint_);
    control_command.set_right_wheel_speed((steering_angle_setpoint_ * wheel_distance_) / 2);
    control_command.set_left_wheel_speed((-steering_angle_setpoint_ * wheel_distance_) / 2);
  }
  if (isCollisionImminent() == EMERGENCY_STOP) {
        control_command.set_front_wheel_angle(0.0);
        control_command.set_right_wheel_speed(0.0);
        control_command.set_left_wheel_speed(0.0);
  }
  return true;
}

float ControllerPathFollow::previewDistSchedule(const proto::PreviewPoint& preview_point, float cmd) {
    float dist = 0.0;
    float cmd_abs = path_follow_cfg_.look_ahea_dis_ratio * std::fabs(cmd);
    dist = std::max(path_follow_cfg_.min_look_ahead_dis, cmd_abs);
    return dist;
}

int ControllerPathFollow::isCollisionImminent (const geometry_msgs::PoseStamped& robot_pose,
                                               const double& linear_vel, const double& angular_vel,
                                               const proto::AnchorBox anchor_boxes) {
    if (anchor_boxes.anchor_pose_size() < 3) {
        ROS_ERROR("Box size is less 3.");
    }
    emergency_polygon_.polygon.points.clear();
    cushion_polygon_.polygon.points.clear();
    replan_polygon_.polygon.points.clear();
    proto::Vector3f robot_state;

    robot_state.set_x(robot_state_.mutable_position()->x());
    robot_state.set_y(robot_state_.mutable_position()->y());
    robot_state.set_z(robot_state_.mutable_roll_pitch_yaw()->z());

    std::vector<geometry_msgs::PolygonStamped> polygons;
    // for (int i = 0; i < anchor_boxes.anchor_pose_size(); ++i) {
    //     geometry_msgs::PolygonStamped polygon;
    //     generateAreaPolygon(anchor_boxes.anchor_pose(i).pose(), polygon.polygon);
    //     polygons.push_back(polygon);
    //     if (inCollision(polygon)) {
    //         return EMERGENCY_STOP;
    //     }
    // }

    generateAreaPolygon(robot_state, emergency_polygon_.polygon);
    generateAreaPolygon(anchor_boxes.anchor_pose(1).pose(), cushion_polygon_.polygon);
    generateAreaPolygon(anchor_boxes.anchor_pose(2).pose(), replan_polygon_.polygon);

    trajectory_viewer_.visualizeAnchorBoxes(polygons, path_follow_cfg_.global_frame);
    trajectory_viewer_.visualizeAnchorBoxes(emergency_polygon_, cushion_polygon_, replan_polygon_, path_follow_cfg_.global_frame);

    transProto2GeometryMsgsPose(anchor_boxes.anchor_pose(anchor_boxes.anchor_pose().size() - 1), replan_goal_);
    replan_goal_index_ = anchor_boxes.anchor_pose(anchor_boxes.anchor_pose().size() - 1).index();

    // trajectory_viewer_.visualizeLocalPlanGoal(replan_pose_.goal(), "map");
    if (inCollision(emergency_polygon_)) {
        ROS_WARN("In emergency collision, emergency stop.");
        collisioned_ = true;
        return EMERGENCY_STOP;
    }
    if (inCollision(cushion_polygon_)) {
        ROS_WARN("In cushion collision, stop with make plan.");
        collisioned_ = true;
        return CUSHION;
    }
    if (inCollision(replan_polygon_)) {
        ROS_WARN("In cushion collision, slow down and make plan.");
        return REPLAN;
    }
    return 0;
}
int ControllerPathFollow::isCollisionImminent() {
    geometry_msgs::PoseStamped pose;
    replan_start_.position.x = robot_state_.position().x();
    replan_start_.position.y = robot_state_.position().y();
    replan_start_.position.z = robot_state_.position().z();
    replan_start_.orientation.x = robot_state_.quaternion().x();
    replan_start_.orientation.y = robot_state_.quaternion().y();
    replan_start_.orientation.z = robot_state_.quaternion().z();
    replan_start_.orientation.w = robot_state_.quaternion().w();

    // if (replan_count_ > 10) {
    //     replan_start_.orientation.x = robot_state_.quaternion().x();
    //     replan_start_.orientation.y = robot_state_.quaternion().y();
    //     replan_start_.orientation.z = robot_state_.quaternion().z();
    //     replan_start_.orientation.w = robot_state_.quaternion().w();
    // } else {
    //     float yaw = robot_state_.roll_pitch_yaw().z();
    //     //TODO(czk) you should set params in yaml file
    //     std::vector<float> yaw_list{-1.04, 1.04, -2.09, 2.09, 3.14};
    //     yaw = yaw + yaw_list[2 * yaw_list.size() - replan_count_];

    //     yaw = normalizeAngle(yaw);
    //     geometry_msgs::Quaternion qua = tf::createQuaternionMsgFromYaw(yaw);

    //     replan_start_.orientation.x = qua.x;
    //     replan_start_.orientation.y = qua.y;
    //     replan_start_.orientation.z = qua.z;
    //     replan_start_.orientation.w = qua.w;
    // }

    emergency_polygon_.polygon.points.clear();
    cushion_polygon_.polygon.points.clear();
    replan_polygon_.polygon.points.clear();
    proto::Vector3f robot_state;

    robot_state.set_x(robot_state_.mutable_position()->x());
    robot_state.set_y(robot_state_.mutable_position()->y());
    robot_state.set_z(robot_state_.mutable_roll_pitch_yaw()->z());
    generateAreaPolygon(robot_state, emergency_polygon_.polygon);
    generateAreaPolygon(robot_state, cushion_polygon_.polygon);
    generateAreaPolygon(robot_state, replan_polygon_.polygon);

    trajectory_viewer_.visualizeAnchorBoxes(emergency_polygon_, cushion_polygon_, replan_polygon_, path_follow_cfg_.global_frame);

    if (inCollision(emergency_polygon_)) {
        collisioned_ = true;
        return EMERGENCY_STOP;
    }

    if (inCollision(cushion_polygon_)) {
        collisioned_ = true;
        return CUSHION;
    }

    if (inCollision(replan_polygon_)) {
        collisioned_ = true;
        return REPLAN;
    }
    replan_flag_ = false;
    return 0;
}
void ControllerPathFollow::transProto2GeometryMsgsPose (const proto::Pose proto_pose, geometry_msgs::Pose& ros_pose) {
    ros_pose.position.x = proto_pose.pose().x();
    ros_pose.position.y = proto_pose.pose().y();
    ros_pose.position.z = 0.0;

    ros_pose.orientation.x = proto_pose.qua().x();
    ros_pose.orientation.y = proto_pose.qua().y();
    ros_pose.orientation.z = proto_pose.qua().z();
    ros_pose.orientation.w = proto_pose.qua().w();
}

bool ControllerPathFollow::inCollision(const geometry_msgs::PolygonStamped& polygon) {
  std::vector<navit_costmap::MapLocation> map_polygon;
  for(unsigned int i = 0; i < polygon.polygon.points.size(); ++i) {
    navit_costmap::MapLocation loc;
    if (!costmap_->worldToMap(polygon.polygon.points[i].x, polygon.polygon.points[i].y, loc.x, loc.y)) {
      //ROS_ERROR("Polygon lies outside map bounds, so we can't fill it");
      return false;
    }
    map_polygon.push_back(loc);
  }
  std::vector<navit_costmap::MapLocation> polygon_cells;
  // get the cells that fill the polygon

  auto map_mutex = costmap_ros_ptr_->getCostmap()->getMutex();
  boost::recursive_mutex::scoped_lock lock(*map_mutex);
  costmap_ptr_ = std::make_shared<navit_costmap::Costmap2D>(*costmap_ros_ptr_->getCostmap());
  costmap_ptr_->convexFillCells(map_polygon, polygon_cells);
  // set the cost of those cells
  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    unsigned char cost = costmap_->getCost(polygon_cells[i].x, polygon_cells[i].y);
    if (cost >= navit_costmap::LETHAL_OBSTACLE) {// INSCRIBED_INFLATED_OBSTACLE
      return true;
    }
  }
  return false;
}

void ControllerPathFollow::findCornerPoint(const proto::Vector3f& robot_state, float x, float y, geometry_msgs::Point32 *p3f) {
    p3f->x = robot_state.x() + x * cos(robot_state.z()) - y * sin(robot_state.z());
    p3f->y = robot_state.y() + x * sin(robot_state.z()) + y * cos(robot_state.z());
}

bool ControllerPathFollow::generateAreaPolygon(const proto::Vector3f& robot_state,
                                               geometry_msgs::Polygon& robot_polygon) {
  geometry_msgs::Point32 corner_point;

  findCornerPoint(robot_state, path_follow_cfg_.robot_box.front_left_x, path_follow_cfg_.robot_box.front_left_y, &corner_point);
  robot_polygon.points.push_back(corner_point);

  findCornerPoint(robot_state, path_follow_cfg_.robot_box.front_right_x, path_follow_cfg_.robot_box.front_right_y, &corner_point);
  robot_polygon.points.push_back(corner_point);

  findCornerPoint(robot_state, path_follow_cfg_.robot_box.rear_right_x, path_follow_cfg_.robot_box.rear_right_y, &corner_point);
  robot_polygon.points.push_back(corner_point);

  findCornerPoint(robot_state, path_follow_cfg_.robot_box.rear_left_x, path_follow_cfg_.robot_box.rear_left_y, &corner_point);
  robot_polygon.points.push_back(corner_point);

  findCornerPoint(robot_state, path_follow_cfg_.robot_box.front_left_x, path_follow_cfg_.robot_box.front_left_y, &corner_point);
  robot_polygon.points.push_back(corner_point);
  return true;
}

nav_msgs::Path ControllerPathFollow::resamplePath(const nav_msgs::Path& input_path, double desired_spacing) {
    nav_msgs::Path output_path;
    output_path.header = input_path.header;


    if(input_path.poses.size() < 2) {
        ROS_ERROR("Input");
        return output_path;
    }

    output_path.poses.push_back(input_path.poses[0]);
    double excess = 0.0;

    size_t i = 1;
    geometry_msgs::PoseStamped last_added = input_path.poses[0];

    while (i < input_path.poses.size()) {
        double d = distance(last_added, input_path.poses[i]);

        if (d + excess < desired_spacing) {
            excess += d;
            ++i;
        } else {
            double ratio = (desired_spacing - excess) / d;
            geometry_msgs::PoseStamped interpolated_pose = interpolate(last_added, input_path.poses[i], ratio);

            output_path.poses.push_back(interpolated_pose);
            last_added = interpolated_pose;

            if (ratio == 1.0) {
                ++i;
            }

            excess = 0.0;
        }
    }
    return output_path;
}

}// namespace control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(control::ControllerPathFollow, navit_core::Controller)
