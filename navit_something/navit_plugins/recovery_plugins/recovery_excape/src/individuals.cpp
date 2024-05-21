#include <recovery_excape/individuals.h>
#include <recovery_excape/toolkit.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>
#include "angles/angles.h"

namespace recovery_excape {

Individuals::Individuals() {}

Individuals::Individuals(const HereditaryInformation info)
  : hereditary_info_{info}, ancestor_{info.front()} {}

Individuals::Individuals(const Individuals& indiv)
{
  hereditary_info_ = indiv.hereditary_info_;
  ancestor_ = indiv.ancestor_;
}

Individuals& Individuals::operator=(const Individuals& indiv)
{
  if (this != &indiv) { // 避免重复赋值
    hereditary_info_ = indiv.hereditary_info_;
    ancestor_ = indiv.ancestor_;
  }
  return *this; // 支持连续等号赋值 A = B = C ...
}

Individuals::Individuals(Individuals&& indiv) noexcept
{
  hereditary_info_=std::move(indiv.hereditary_info_);
  ancestor_=std::move(indiv.ancestor_);
}

Individuals& Individuals::operator=(const Individuals&& indiv) noexcept
{
  if (this != &indiv) { // 避免重复赋值
    hereditary_info_=std::move(indiv.hereditary_info_);
    ancestor_=std::move(indiv.ancestor_);
  }
  return *this; // 支持连续等号赋值 A = B = C ...
}

void Individuals::Reproduce()
{
  size_t n = hereditary_info_.front().size();
  for (size_t i = 0; i < n; i ++) {
    size_t ri = SelectRandomGeno();
    hereditary_info_.at(ri).at(i).Reproduce();    
  }
}

void Individuals::Mutation()
{
  size_t n = hereditary_info_.front().size();
  for (size_t i = 0; i < n; i ++) {
    size_t ri = SelectRandomGeno();
    hereditary_info_.at(ri).at(i).Mutation();    
  }
}

std::vector<Chromosome> Individuals::GetHereditaryInformation() const
{
  return hereditary_info_;
}

Individuals Individuals::Cross(const Individuals& indiv)
{
  auto other_info = indiv.GetHereditaryInformation();
  HereditaryInformation hybird_info(hereditary_info_.size());
  for (size_t i = 0; i < hybird_info.size(); i ++) {
    if (i%2) hybird_info.at(i) = hereditary_info_.at(i);
    else hybird_info.at(i) = other_info.at(i);
  }
  return Individuals(hybird_info);
}

std::vector<geometry_msgs::Pose> Individuals::GenCharacter(const geometry_msgs::PoseStamped pose,
  const navit_costmap::Costmap2D& map, const double v, const double wheel_base,
  const bool use_steering)
{
  std::vector<geometry_msgs::Pose> poses;
  geometry_msgs::Pose p = pose.pose;
  poses.push_back(p);
  // int continue_move_back = 5, move_back_step = 0;
  int dir = v > 0 ? 1 : -1;
  for (auto chronosome : hereditary_info_) {
    double ws;
    GetBehavior(chronosome, p, map, wheel_base, ws, dir);
    // if (v_signed < 0) move_back_step = continue_move_back;
    // if (move_back_step -- > 0) v_signed = -1;
    p = Predict(p, v, ws, chronosome.at(1).GetGeno(), wheel_base, use_steering);
    poses.push_back(p);
  }
  return poses;
}

void Individuals::Survive()
{
  hereditary_info_.erase(hereditary_info_.begin());
  auto back = hereditary_info_.back();
  hereditary_info_.push_back(ancestor_);
}

void Individuals::GetBehavior(const Chromosome& chro, const geometry_msgs::Pose p,
  const navit_costmap::Costmap2D& map, const double wheel_base, double& ws, int& v_signed)
{
  double yaw = tf::getYaw(p.orientation), yaw_offset = 0.54;
  double left_yaw = angles::normalize_angle(yaw + yaw_offset);
  double right_yaw = angles::normalize_angle(yaw - yaw_offset);
  double hpx = p.position.x + cos(yaw) * wheel_base * v_signed;
  double hpy = p.position.y + sin(yaw) * wheel_base * v_signed;
  double lpx = p.position.x + cos(left_yaw) * wheel_base * v_signed;
  double lpy = p.position.y + sin(left_yaw) * wheel_base * v_signed;
  double rpx = p.position.x + cos(right_yaw) * wheel_base * v_signed;
  double rpy = p.position.y + sin(right_yaw) * wheel_base * v_signed;
  double base_cost = 255.0, head_cost = 255.0, left_cost = 255.0, right_cost = 255.0, back_cost = 240;

  unsigned int mx, my;
  if (map.worldToMap(p.position.x, p.position.y, mx, my)) {
    base_cost = map.getCost(mx, my);
  }
  if (map.worldToMap(hpx, hpy, mx, my)) {
    head_cost = map.getCost(mx, my);
  }
  if (map.worldToMap(lpx, lpy, mx, my)) {
    left_cost = map.getCost(mx, my);
  }
  if (map.worldToMap(rpx, rpy, mx, my)) {
    right_cost = map.getCost(mx, my);
  }

  // v_signed = head_cost >= back_cost ? -v_signed : v_signed;
  int ws_signed = left_cost < right_cost ? 1 : -1;
  double ws_degree = left_cost < right_cost ? right_cost : left_cost;  
  double ws_base = 1.0;
  ws = ws_base * ws_degree / 255.0 * ws_signed * chro.at(0).GetGeno();
}

// -------------------- protected --------------------

size_t Individuals::SelectRandomGeno()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dist(0, hereditary_info_.size()-1);
  size_t random = dist(gen);
  return random;
}



// -------------------- private --------------------




} // namespace recovery_excape
