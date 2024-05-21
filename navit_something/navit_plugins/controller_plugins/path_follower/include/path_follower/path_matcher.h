//
// Created by yjh on 23-2-27.
//
#ifndef SRC_PATHMATCHER_H
#define SRC_PATHMATCHER_H
#include <ros/ros.h>
#include <tf/tf.h>

#include "robot_state.pb.h"
#include "config_control.pb.h"
#include "control_command.pb.h"
#include "path_follow_cfg.pb.h"
#include "path.pb.h"
#include "geometry2.h"

namespace control{
typedef google::protobuf::internal::RepeatedPtrIterator<const proto::PathPoint> PathPointIter;

class PathMatcher {
public:
    struct SteerError {
        float lat;
        float lon;
        float theta;
        float yaw_rate;
    };
    // matched point on path
    struct MatchedPointsIter {
        PathPointIter rear_axis_iter;
        PathPointIter anchor_point_iter;
        PathPointIter fb_point_iter;
        // PathPointIter ff_point_iter;
    };
    PathMatcher(float point_on_path_threshold) : point_on_path_threshold_(point_on_path_threshold) {
    }

    ~PathMatcher() = default;

    bool findNearestPoint(const proto::Path& path, const geometry2::Vec2f current_pose, PathPointIter* nearest_point_iter,
                          float* min_squared_dist, int* index);
    bool findNearestPoint(const proto::Path& path, const geometry2::Vec2f current_pose, PathPointIter* nearest_point_iter, int* index);

    bool findInCiclePoint(const proto::Path &path, const geometry2::Vec2f current_pose, PathPointIter *nearest_point_iter, const float dis, int current_index);

    bool updateMatchedPoints(const proto::Path& path, const proto::RobotState& robot_state, float look_ahead_dist,
                             bool reverse, geometry2::Vec2f& anchor_point_ref, int* index, const float box_size, const float iter_num,
                             proto::AnchorBox& anchor_boxes, bool& should_rotate, int& rotate_index);

    float getRobotTargetDistSquare(const geometry2::Vec2f current_pose) const;

    inline SteerError getSteerErr() { return steer_error_; }
private:
    float point_on_path_threshold_;
    SteerError steer_error_;
    MatchedPointsIter matched_points_iter_;
    // DISALLOW_COPY_AND_ASSIGN(PathMatcher);
};
}// namespace control

#endif //SRC_PATHMATCHER_H
