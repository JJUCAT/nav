//
// Created by yjh on 23-2-24.
//
#include <path_planner/astar_search.h>

namespace planner {
AstarSearch::AstarSearch() : node_initialized_(false) { ros::NodeHandle private_nh_("~"); }

AstarSearch::~AstarSearch() {}

bool AstarSearch::init(proto::ConfigGraphSearch &graph_search_parameters) {
  ros::NodeHandle nh;

  nh.param("astar_search/angle_size", avoidance_cfg_.angle_size, 40);
  nh.param("astar_search/minimum_turning_radius", avoidance_cfg_.minimum_turning_radius, 0.1);
  nh.param("astar_search/goal_radius", avoidance_cfg_.goal_radius, 3.0);
  nh.param("astar_search/goal_angle", avoidance_cfg_.goal_angle, 80.0);
  nh.param("astar_search/use_back", avoidance_cfg_.use_back, true);
  nh.param("astar_search/robot_length", avoidance_cfg_.robot_length, 0.2);
  nh.param("astar_search/robot_width", avoidance_cfg_.robot_width, 0.2);
  nh.param("astar_search/base2back", avoidance_cfg_.base2back, 0.1);
  nh.param("astar_search/curve_weight", avoidance_cfg_.curve_weight, 2.54);
  nh.param("astar_search/reverse_weight", avoidance_cfg_.reverse_weight, 2.5);
  nh.param("astar_search/use_wavefront_heuristic", avoidance_cfg_.use_wavefront_heuristic, false);

  path_frame_ = "map";
  angle_size_ = avoidance_cfg_.angle_size;
  minimum_turning_radius_ = avoidance_cfg_.minimum_turning_radius;
  goal_radius_init_ = avoidance_cfg_.goal_radius;
  goal_angle_init_ = avoidance_cfg_.goal_angle;
  use_back_inital_ = avoidance_cfg_.use_back;
  robot_length_ = avoidance_cfg_.robot_length;
  robot_width_ = avoidance_cfg_.robot_width;
  base2back_ = avoidance_cfg_.base2back;
  curve_weight_ = avoidance_cfg_.curve_weight;
  reverse_weight_ = avoidance_cfg_.reverse_weight;
  use_wavefront_heuristic_ = avoidance_cfg_.use_wavefront_heuristic;

  createStateUpdateTable(angle_size_);
  return true;
}
bool AstarSearch::init(std::string name) {
  ros::NodeHandle nh("~/" + name);

  nh.param("astar_search/angle_size", avoidance_cfg_.angle_size, 40);
  nh.param("astar_search/minimum_turning_radius", avoidance_cfg_.minimum_turning_radius, 0.1);
  nh.param("astar_search/goal_radius", avoidance_cfg_.goal_radius, 0.3);
  nh.param("astar_search/goal_angle", avoidance_cfg_.goal_angle, 80.0);
  nh.param("astar_search/use_back", avoidance_cfg_.use_back, false);
  nh.param("astar_search/robot_length", avoidance_cfg_.robot_length, 1.0);
  nh.param("astar_search/robot_width", avoidance_cfg_.robot_width, 1.6);
  nh.param("astar_search/base2back", avoidance_cfg_.base2back, 0.1);
  nh.param("astar_search/curve_weight", avoidance_cfg_.curve_weight, 2.54);
  nh.param("astar_search/reverse_weight", avoidance_cfg_.reverse_weight, 2.5);
  nh.param("astar_search/use_wavefront_heuristic", avoidance_cfg_.use_wavefront_heuristic, true);

  path_frame_ = "map";
  angle_size_ = avoidance_cfg_.angle_size;
  minimum_turning_radius_ = avoidance_cfg_.minimum_turning_radius;
  goal_radius_init_ = avoidance_cfg_.goal_radius;
  goal_angle_init_ = avoidance_cfg_.goal_angle;
  use_back_inital_ = avoidance_cfg_.use_back;
  robot_length_ = avoidance_cfg_.robot_length;
  robot_width_ = avoidance_cfg_.robot_width;
  base2back_ = avoidance_cfg_.base2back;
  curve_weight_ = avoidance_cfg_.curve_weight;
  reverse_weight_ = avoidance_cfg_.reverse_weight;
  use_wavefront_heuristic_ = avoidance_cfg_.use_wavefront_heuristic;

  createStateUpdateTable(angle_size_);
}

void AstarSearch::resizeNode(int width, int height, int angle_size) {
  nodes_.resize(height);
  for (int i = 0; i < height; i++) nodes_[i].resize(width);
  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++) nodes_[i][j].resize(angle_size);
}

bool AstarSearch::getPath(const double &angle, proto::Polyline *path) {
  proto::Vector3f v3f;
  double direction = 1;
  if (path_.poses.size() < 3) return false;
  for (unsigned int i = 0; i < path_.poses.size(); i++) {
    v3f.set_x(path_.poses[i].pose.position.x);
    v3f.set_y(path_.poses[i].pose.position.y);
    path->add_points()->CopyFrom(v3f);
    if (i < 2) continue;
    geometry2::Vec2f point_l = geometry2::Vec2f(path_.poses[i - 2].pose.position.x, path_.poses[i - 2].pose.position.y);
    geometry2::Vec2f point_m = geometry2::Vec2f(path_.poses[i - 1].pose.position.x, path_.poses[i - 1].pose.position.y);
    geometry2::Vec2f point_r = geometry2::Vec2f(path_.poses[i].pose.position.x, path_.poses[i].pose.position.y);
    double path_angle = (point_r - point_m).heading();
    if (std::fabs(normalizeAngle(angle - path_angle)) > M_PI / 2) return false;
    if (direction > 0) direction = (point_m - point_l).dot(point_r - point_m);
  }
  double s = 0;
  for (int i = 1; i < path->points_size(); i++) s = s + geometry2::Vec2f(path->points(i)).distance(geometry2::Vec2f(path->points(i - 1)));
  if (s < 2.0) return false;
  if (direction < 0)
    return false;
  else
    return true;
}
bool AstarSearch::getPath(proto::Polyline* local_path) {
  proto::Vector3f v3f;
  for (unsigned int i = 0; i < path_.poses.size(); i++) {
    v3f.set_x(path_.poses[i].pose.position.x);
    v3f.set_y(path_.poses[i].pose.position.y);
    if (i >= 1) {
        v3f.set_z(normalizeAngle(atan2((path_.poses[i].pose.position.y - path_.poses[i-1].pose.position.y)
                                      ,(path_.poses[i].pose.position.x - path_.poses[i-1].pose.position.x))));
    } else {
        v3f.set_z(normalizeAngle(atan2((path_.poses[0].pose.position.y - path_.poses[1].pose.position.y)
                                      ,(path_.poses[0].pose.position.x - path_.poses[1].pose.position.x))));
    }
    local_path->add_points()->CopyFrom(v3f);
  }
  return true;
}

void AstarSearch::poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta) {
  *index_y = (pose.position.x - map_info_.origin.position.x)/ map_info_.resolution;
  *index_x = (pose.position.y - map_info_.origin.position.y)/ map_info_.resolution;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose.orientation, quat);

  double yaw = tf::getYaw(quat);
  if (yaw < 0) yaw += 2 * M_PI;
  // Descretize angle
  static double one_angle_range = 2 * M_PI / angle_size_;
  *index_theta = yaw / one_angle_range;
  *index_theta %= angle_size_;
}

void AstarSearch::setTolerance(const int &type) {
  goal_angle_ = goal_angle_init_;
  if (type == 3) {
    goal_radius_ = 0.1;
    goal_angle_ = 5;
    use_back_ = false;
  } else if (type == 2) {
    goal_radius_ = 0.5;
    use_back_ = use_back_inital_;
  } else {

    goal_radius_ = goal_radius_init_;
    use_back_ = use_back_inital_;
  }
  // VERROR("use_back_ "<<use_back_<<" type "<<type);
}

void AstarSearch::createStateUpdateTable(int angle_size) {

  use_back_ = false;
  // Vehicle moving for each angle
  state_update_table_.resize(angle_size);
  // 6 is the number of steering actions
  // max-left, no-turn, and max-right of forward and backward
  if (use_back_) {
    for (int i = 0; i < angle_size; i++) state_update_table_[i].resize(6);
  } else {
    for (int i = 0; i < angle_size; i++) state_update_table_[i].resize(3);
  }

  // Minimum moving distance with one state update
  //     arc  = r                       * theta
  double step = minimum_turning_radius_ * (2.0 * M_PI / angle_size_);

  for (int i = 0; i < angle_size; i++) {
    double descretized_angle = 2.0 * M_PI / angle_size;
    double robot_angle = descretized_angle * i;

    // Calculate right and left circle
    // Robot moves along these circles
    double right_circle_center_x = minimum_turning_radius_ * std::sin(robot_angle);
    double right_circle_center_y = minimum_turning_radius_ * std::cos(robot_angle) * -1.0;
    double left_circle_center_x = right_circle_center_x * -1.0;
    double left_circle_center_y = right_circle_center_y * -1.0;

    // Calculate x and y shift to next state
    NodeUpdate nu;
    // forward
    nu.shift_x = step * std::cos(robot_angle);
    nu.shift_y = step * std::sin(robot_angle);
    nu.rotation = 0;
    nu.index_theta = 0;
    nu.step = step;
    nu.curve = false;
    nu.back = false;
    state_update_table_[i][0] = nu;

    // forward right
    nu.shift_x = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + robot_angle - descretized_angle);
    nu.shift_y = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + robot_angle - descretized_angle);
    nu.rotation = descretized_angle * -1.0;
    nu.index_theta = -1;
    nu.step = step;
    nu.curve = true;
    nu.back = false;
    state_update_table_[i][1] = nu;

    // forward left
    nu.shift_x = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + robot_angle + descretized_angle);
    nu.shift_y = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + robot_angle + descretized_angle);
    nu.rotation = descretized_angle;
    nu.index_theta = 1;
    nu.step = step;
    nu.curve = true;
    nu.back = false;
    state_update_table_[i][2] = nu;

    // We don't use back move
    if (!use_back_) continue;

    // backward
    nu.shift_x = step * std::cos(robot_angle) * -1.0;
    nu.shift_y = step * std::sin(robot_angle) * -1.0;
    nu.rotation = 0;
    nu.index_theta = 0;
    nu.step = step;
    nu.curve = false;
    nu.back = true;
    state_update_table_[i][3] = nu;

    // backward right
    nu.shift_x = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + robot_angle + descretized_angle);
    nu.shift_y = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + robot_angle + descretized_angle);
    nu.rotation = descretized_angle;
    nu.index_theta = 1;
    nu.step = step;
    nu.curve = true;
    nu.back = true;
    state_update_table_[i][4] = nu;

    // backward left
    nu.shift_x = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + robot_angle - descretized_angle);
    nu.shift_y = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + robot_angle - descretized_angle);
    nu.rotation = descretized_angle * -1.0;
    nu.index_theta = -1;
    nu.step = step;
    nu.curve = true;
    nu.back = true;
    state_update_table_[i][5] = nu;
  }
}

bool AstarSearch::isOutOfRange(int index_x, int index_y) {
  if (index_x < 0 || index_x >= static_cast<int>(map_info_.width) || index_y < 0 || index_y >= static_cast<int>(map_info_.height))
    return true;

  return false;
}

void AstarSearch::setPath(const SimpleNode &goal) {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = path_frame_;
  path_.header = header;

  // From the goal node to the start node
  AstarNode *node = &nodes_[goal.index_y][goal.index_x][goal.index_theta];

  while (node != NULL) {
    // Set tf pose
    tf::Vector3 origin(node->x, node->y, 0);
    tf::Pose tf_pose;
    tf_pose.setOrigin(origin);
    tf_pose.setRotation(tf::createQuaternionFromYaw(node->theta));

    // Transform path to global frame
    // tf_pose = map2ogm_ * tf_pose;

    // Set path as ros message
    geometry_msgs::PoseStamped ros_pose;
    tf::poseTFToMsg(tf_pose, ros_pose.pose);
    ros_pose.header = header;
    path_.poses.push_back(ros_pose);

    //----DEBUG-------
    /*
    debug_pose_array_.poses.push_back(ros_pose.pose);
    int index_theta = node->theta / (2.0 * M_PI / angle_size_);
    int index_x = node->x / map_info_.resolution;
    int index_y = node->y / map_info_.resolution;
    SimpleNode sn(index_x, index_y, index_theta, 0, 0);
    */
    // detectCollisionPose(sn);
    //--------------

    // To the next node
    node = node->parent;
  }

  // Reverse the vector to be start to goal order
  std::reverse(path_.poses.begin(), path_.poses.end());
}

// theta is 0 ~ 2pi
bool AstarSearch::isGoal(double x, double y, double theta) {
  // To reduce computation time, we use square value
  static const double goal_radius = goal_radius_ * goal_radius_;  // meter
  static const double goal_angle = M_PI * goal_angle_ / 180.0;    // degrees -> radian

  // Check the pose of goal
  if (std::pow(goal_pose_local_.pose.position.x - x, 2) + std::pow(goal_pose_local_.pose.position.y - y, 2) < goal_radius) {
    // Get goal yaw in 0 ~ 2pi
    double goal_yaw = modifyTheta(tf::getYaw(goal_pose_local_.pose.orientation));

    // Check the orientation of goal
    if (calcDiffOfRadian(goal_yaw, theta) < goal_angle) return true;
  }

  return false;
}

bool AstarSearch::detectCollision(const SimpleNode &sn, bool start) {
  // Define the robot as rectangle
  static double left = -1.0 * base2back_;
  static double right = robot_length_ - base2back_;
  static double top = robot_width_ / 2.0;
  static double bottom = -1.0 * robot_width_ / 2.0;
  static double resolution = map_info_.resolution;

  // ROS_INFO(" left %f,   right  %f  ,   top    %f  , bottom   %f",left,right,top,bottom);

  // Coordinate of base_link in OccupancyGrid frame
  static double one_angle_range = 2.0 * M_PI / angle_size_;
  double base_x = sn.index_x * resolution;
  double base_y = sn.index_y * resolution;
  double base_theta = sn.index_theta * one_angle_range;

  // Calculate cos and sin in advance
  double cos_theta = std::cos(base_theta);
  double sin_theta = std::sin(base_theta);

  // Convert each point to index and check if the node is Obstacle
  int size = 0;
  // if (!start) {
    for (double x = left; x < right; x += resolution) {
      for (double y = top; y > bottom; y -= resolution) {
        // 2D point rotation
        size ++;
        int index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
        int index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;

        if (isOutOfRange(index_x, index_y)) return true;
        if (nodes_[index_y][index_x][0].status == STATUS::OBS) {
          //std::cout << "Obs " << std::endl;
          return true;
        }
      }
    }
  // } else {
  //   for (double x = left / 2; x < right / 2; x += resolution) {
  //     for (double y = top / 2; y > bottom / 2; y -= resolution) {
  //       // 2D point rotation
  //       int index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
  //       int index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;
  //       if (isOutOfRange(index_x, index_y)) return true;
  //       if (nodes_[index_y][index_x][0].status == STATUS::OBS) return true;
  //     }
  //   }
  // }

  return false;
}

bool AstarSearch::calcWaveFrontHeuristic(const SimpleNode &sn) {
  // Set start point for wavefront search
  // This is goal for Astar search
  nodes_[sn.index_y][sn.index_x][0].hc = 0;
  WaveFrontNode wf_node(sn.index_x, sn.index_y, 1e-10);
  std::queue<WaveFrontNode> qu;
  qu.push(wf_node);

  // State update table for wavefront search
  // Nodes are expanded for each neighborhood cells (moore neighborhood)
  double resolution = map_info_.resolution;
  static std::vector<WaveFrontNode> updates = {
      getWaveFrontNode(0, 1, resolution),
      getWaveFrontNode(-1, 0, resolution),
      getWaveFrontNode(1, 0, resolution),
      getWaveFrontNode(0, -1, resolution),
      getWaveFrontNode(-1, 1, std::hypot(resolution, resolution)),
      getWaveFrontNode(1, 1, std::hypot(resolution, resolution)),
      getWaveFrontNode(-1, -1, std::hypot(resolution, resolution)),
      getWaveFrontNode(1, -1, std::hypot(resolution, resolution)),
  };

  // Get start index
  int start_index_x;
  int start_index_y;
  int start_index_theta;
  poseToIndex(start_pose_local_.pose, &start_index_x, &start_index_y, &start_index_theta);

  // Whether the robot can reach goal
  bool reachable = false;

  // Start wavefront search
  while (!qu.empty()) {
    WaveFrontNode ref = qu.front();
    qu.pop();

    WaveFrontNode next;
    for (const auto &u : updates) {
      next.index_x = ref.index_x + u.index_x;
      next.index_y = ref.index_y + u.index_y;

      // out of range OR already visited OR obstacle node
      if (isOutOfRange(next.index_x, next.index_y) || nodes_[next.index_y][next.index_x][0].hc > 0 ||
          nodes_[next.index_y][next.index_x][0].status == STATUS::OBS)
        continue;

      // Take the size of robot into account
      if (detectCollisionWaveFront(next)) continue;

      // Check if we can reach from start to goal
      if (next.index_x == start_index_x && next.index_y == start_index_y) reachable = true;

      // Set wavefront heuristic cost
      next.hc = ref.hc + u.hc;
      nodes_[next.index_y][next.index_x][0].hc = next.hc;

      qu.push(next);
    }
  }

  // End of search
  return reachable;
}

// Simple collidion detection for wavefront search
bool AstarSearch::detectCollisionWaveFront(const WaveFrontNode &ref) {
  // Define the robot as square
  static double half = robot_width_ / 2;
  double robot_x = ref.index_x * map_info_.resolution;
  double robot_y = ref.index_y * map_info_.resolution;

  for (double y = half; y > -1.0 * half; y -= map_info_.resolution) {
    for (double x = -1.0 * half; x < half; x += map_info_.resolution) {
      int index_x = (robot_x + x) / map_info_.resolution;
      int index_y = (robot_y + y) / map_info_.resolution;

      if (isOutOfRange(index_x, index_y)) return true;

      if (nodes_[index_y][index_x][0].status == STATUS::OBS) return true;
    }
  }

  return false;
}

void AstarSearch::reset() {
  // Clear path
  path_.poses.clear();

  // Clear queue
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> empty;
  std::swap(openlist_, empty);
}

void AstarSearch::setMap(const nav_msgs::OccupancyGrid &map) {
  map_info_ = map.info;

  // TODO: what frame do we use?
  std::string map_frame = "odom";
  std::string ogm_frame = map.header.frame_id;
  // Set transform
  tf::StampedTransform map2ogm_frame;
  try {
    tf_listener_.lookupTransform(map_frame, ogm_frame, ros::Time(0), map2ogm_frame);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Error failed to acquire quaternion.");
    return;
  }

  tf::Transform map2ogm;
  geometry_msgs::Pose ogm_in_map = transformPose(map_info_.origin, map2ogm_frame);
  tf::poseMsgToTF(ogm_in_map, map2ogm_);

  // Initialize node according to map size
  if (!node_initialized_) {
    resizeNode(map.info.width, map.info.height, angle_size_);
    node_initialized_ = true;
  }
  for (size_t i = 0; i < map.info.height; i++) {
    for (size_t j = 0; j < map.info.width; j++) {
      // Index of subscribing OccupancyGrid message
      size_t og_index = i * map.info.width + j;
      int cost = map.data[og_index];
      // more than threshold or unknown area
      // if (cost > obstacle_threshold_/* || cost < 0 */) {
      if (cost >= 99 /* || cost < 0 */) {
        nodes_[i][j][0].status = STATUS::OBS;
      } else {
        for (int k = 0; k < angle_size_; k++) {
          // nodes_[i][j][k].gc     = 0;
          nodes_[i][j][k].hc = 0;
          nodes_[i][j][k].status = STATUS::NONE;
          nodes_[i][j][k].parent = NULL;
        }
      }
    }
  }
}

void AstarSearch::setMap(const navit_costmap::Costmap2DROS* costmap_ros) {
  if (!costmap_ros) {
    ROS_ERROR("CostmapROS pointer is invalid.");
  }
  navit_costmap::Costmap2D* costmap = costmap_ros->getCostmap();

  map_info_.width = costmap->getSizeInCellsX();
  map_info_.height = costmap->getSizeInCellsY();
  map_info_.origin.position.x = costmap->getOriginX();
  map_info_.origin.position.y = costmap->getOriginY();
  map_info_.origin.orientation.w = 1.0;
  map_info_.origin.orientation.x = 0.0;
  map_info_.origin.orientation.y = 0.0;
  map_info_.origin.orientation.z = 0.0;
  map_info_.resolution = costmap->getResolution();

  std::string map_frame = "odom";
  std::string ogm_frame = costmap_ros->getGlobalFrameID();
  tf::StampedTransform map2ogm_frame;
  try {
    tf_listener_.lookupTransform(map_frame, ogm_frame, ros::Time(0), map2ogm_frame);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Error failed to acquire transform.");
    return;
  }

  tf::Transform map2ogm;
  geometry_msgs::Pose ogm_in_map = transformPose(map_info_.origin, map2ogm_frame);
  tf::poseMsgToTF(ogm_in_map, map2ogm_);

  if (!node_initialized_) {
    resizeNode(map_info_.width, map_info_.height, angle_size_);
    node_initialized_ = true;
  }

  for (size_t i = 0; i < map_info_.height; i++) {
    for (size_t j = 0; j < map_info_.width; j++) {
      unsigned char cost = costmap->getCost(j, i);
      if (cost >= navit_costmap::LETHAL_OBSTACLE /* || cost < 0 */) {
        nodes_[i][j][0].status = STATUS::OBS;
      } else {
        for (int k = 0; k < angle_size_; k++) {
          nodes_[i][j][k].hc = 0;
          nodes_[i][j][k].status = STATUS::NONE;
          nodes_[i][j][k].parent = NULL;
        }
      }
    }
  }
}
bool AstarSearch::setStartNode() {
  // Get index of start pose
  path_.poses.clear();

  int index_x;
  int index_y;
  int index_theta;

  poseToIndex(start_pose_local_.pose, &index_x, &index_y, &index_theta);

  if(isOutOfRange(index_x, index_y)) {
    ROS_ERROR("Out of Range.");
  }

  SimpleNode start_sn(index_x, index_y, index_theta, 0, 0);
  // Check if start is valid

  if(detectCollision(start_sn, true)) {
    ROS_ERROR("Start is Collision");
  }

  if (isOutOfRange(index_x, index_y) || detectCollision(start_sn, true)) return false;

  // Set start node
  AstarNode &start_node = nodes_[index_y][index_x][index_theta];
  start_node.x = start_pose_local_.pose.position.x;
  start_node.y = start_pose_local_.pose.position.y;

  start_node.theta = 2.0 * M_PI / angle_size_ * index_theta;
  start_node.gc = 0;
  start_node.back = false;
  start_node.status = STATUS::OPEN;
  if (!use_wavefront_heuristic_)
    start_node.hc = calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y, goal_pose_local_.pose.position.x,
                                 goal_pose_local_.pose.position.y);

  // Push start node to openlist
  start_sn.cost = start_node.gc + start_node.hc;
  openlist_.push(start_sn);
  return true;
}

bool AstarSearch::setGoalNode() {
  // Get index of goal pose
  path_.poses.clear();
  int index_x;
  int index_y;
  int index_theta;


  poseToIndex(goal_pose_local_.pose, &index_x, &index_y, &index_theta);

  if(isOutOfRange(index_x, index_y)) {
    ROS_ERROR("Goal Out of Range.");
  }

  SimpleNode goal_sn(index_x, index_y, index_theta, 0, 0);
  // Check if goal is valid
  if (isOutOfRange(index_x, index_y) || detectCollision(goal_sn)) return false;

  // Calculate wavefront heuristic cost
  if (use_wavefront_heuristic_) {
    bool wavefront_result = calcWaveFrontHeuristic(goal_sn);
    if (!wavefront_result) {
      ROS_ERROR("Goal is not reachable...");
      return false;
    }
  }

  return true;
}

bool AstarSearch::search() {
  path_.poses.clear();
  //while (!openlist_.empty()) openlist_.pop();
  // auto start = std::chrono::system_clock::now();
  ros::WallTime start = ros::WallTime::now();
  // -- Start Astar search ----------
  // If the openlist is empty, search failed
  while (!openlist_.empty()) {
    // Terminate the search if the count reaches a certain value
    static int search_count = 0;
    search_count ++;
    if (search_count > 800000 || ((ros::WallTime::now() - start).toSec()) > 1.0 || break_search_flag_) {
      // VERROR("Exceed time limit: " << (ros::WallTime::now() - start).toSec() << " s."
      //                              << " search_count " << search_count << " break_search_flag_ " << break_search_flag_);
      ROS_WARN("search_count is %d", search_count);
      //printf("Cal time is %f", (ros::WallTime::now() - start).toSec());
      search_count = 0;
      break;
      return false;
    }

    // Pop minimum cost node from openlist
    SimpleNode sn;
    sn = openlist_.top();
    openlist_.pop();
    nodes_[sn.index_y][sn.index_x][sn.index_theta].status = STATUS::CLOSED;
    // Expand nodes from this node
    AstarNode *current_node = &nodes_[sn.index_y][sn.index_x][sn.index_theta];

    // for each update
    for (const auto &state : state_update_table_[sn.index_theta]) {
      // Next state
      double next_x = current_node->x + state.shift_x;
      double next_y = current_node->y + state.shift_y;
      double next_theta = modifyTheta(current_node->theta + state.rotation);
      double move_cost = state.step;

      // Display search process
      geometry_msgs::Pose p;
      p.position.x = next_x;
      p.position.y = next_y;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(next_theta), p.orientation);
      // p = transformPose(p, map2ogm_);
      debug_pose_array_.poses.push_back(p);

      // Increase curve cost
      if (state.curve) move_cost *= curve_weight_;

      // Increase reverse cost
      if (current_node->back != state.back) move_cost *= reverse_weight_;

      // Calculate index of the next state
      SimpleNode next;
      next.index_x = (next_x - map_info_.origin.position.x)/ map_info_.resolution;
      next.index_y = (next_y - map_info_.origin.position.y)/map_info_.resolution;
      next.index_theta = sn.index_theta + state.index_theta;
      // Avoid invalid index
      next.index_theta = (next.index_theta + angle_size_) % angle_size_;

      // Check if the index is valid
      if (isOutOfRange(next.index_x, next.index_y) || detectCollision(next)) {
        continue;
      }

      AstarNode *next_node = &nodes_[next.index_y][next.index_x][next.index_theta];
      double next_hc = nodes_[next.index_y][next.index_x][0].hc;
      // Calculate euclid distance heuristic cost
      if (!use_wavefront_heuristic_)
        next_hc = calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y);
      // GOAL CHECK
      if (isGoal(next_x, next_y, next_theta)) {
        search_count = 0;
        next_node->status = STATUS::OPEN;
        next_node->x = next_x;
        next_node->y = next_y;
        next_node->theta = next_theta;
        next_node->gc = current_node->gc + move_cost;
        next_node->hc = next_hc;
        next_node->back = state.back;
        next_node->parent = current_node;

        setPath(next);

        if (path_.poses.size() < 5) {
          ROS_ERROR("Path poses is less than 10.");
          return false;
        }

        if (fabs(path_.poses[0].pose.position.x - start_pose_local_.pose.position.x) < 0.05 &&
            fabs(path_.poses[0].pose.position.y - start_pose_local_.pose.position.y) < 0.05) {
          return true;
        } else {
          return false;
        }
      }
      // NONE
      if (next_node->status == STATUS::NONE) {
        next_node->status = STATUS::OPEN;
        next_node->x = next_x;
        next_node->y = next_y;
        next_node->theta = next_theta;
        next_node->gc = current_node->gc + move_cost;
        next_node->hc = next_hc;
        next_node->back = state.back;
        next_node->parent = current_node;

        next.cost = next_node->gc + next_node->hc;
        openlist_.push(next);
        continue;
      }
      // OPEN or CLOSED
      if (next_node->status == STATUS::OPEN || next_node->status == STATUS::CLOSED) {
        if (current_node->gc + move_cost + next_hc < next_node->gc + next_hc) {
          next_node->status = STATUS::OPEN;
          next_node->x = next_x;
          next_node->y = next_y;
          next_node->theta = next_theta;
          next_node->gc = current_node->gc + move_cost;
          next_node->hc = next_hc;
          next_node->back = state.back;
          next_node->parent = current_node;

          next.cost = next_node->gc + next_node->hc;
          openlist_.push(next);
          continue;
        }
      }
      if (search_count == 0) break;
    }  // state update
  }
  // Failed to find path
  ROS_ERROR("Openlist is Empty!");
  return false;
}

bool AstarSearch::makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose,
                           const nav_msgs::OccupancyGrid &map) {
  // start_pose_local_.pose = start_pose;
  // goal_pose_local_.pose = goal_pose;
  // break_search_flag_ = false;

  // setMap(map);

  // if (!setStartNode()) {
  //   ROS_ERROR("Invalid start pose!");
  //   return false;
  // }

  // if (!setGoalNode()) {
  //   ROS_ERROR("Invalid goal pose!");
  //   return false;
  // }

  // bool result = search();

  return true;
}

bool AstarSearch::makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose,
                           const navit_costmap::Costmap2DROS* costmap_ros) {
  ROS_ERROR_STREAM("start pose is " << start_pose << " goal pose is " << goal_pose);

  start_pose_local_.pose = start_pose;
  goal_pose_local_.pose = goal_pose;
  break_search_flag_ = false;

  setMap(costmap_ros);

  if (!setStartNode()) {
    ROS_ERROR("Invalid start pose!");
    return false;
  }

  if (!setGoalNode()) {
    ROS_ERROR("Invalid goal pose!");
    return false;
  }

  bool result = search();

  return result;
}
// for debug
void AstarSearch::publishPoseArray(const ros::Publisher &pub, const std::string &frame) {
  debug_pose_array_.header.frame_id = frame;
  // debug_pose_array_.poses.push_back(map_info_.origin);
  debug_pose_array_.poses.push_back(start_pose_.pose);
  debug_pose_array_.poses.push_back(goal_pose_.pose);

  pub.publish(debug_pose_array_);

  debug_pose_array_.poses.clear();
}

// for demo
void AstarSearch::broadcastPathTF() {
  tf::Transform transform;

  // Broadcast from start pose to goal pose
  for (unsigned int i = 0; i < path_.poses.size(); i++) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(path_.poses[i].pose.orientation, quat);
    transform.setOrigin(tf::Vector3(path_.poses[i].pose.position.x, path_.poses[i].pose.position.y, path_.poses[i].pose.position.z));
    transform.setRotation(quat);

    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/astar_path"));

    // sleep 0.1 [sec]
    usleep(100000);
  }
}
} //namespace planner

