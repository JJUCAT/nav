//
// Created by yjh on 23-2-27.
//
#include "path_follower/path_matcher.h"
namespace control {
    bool PathMatcher::findNearestPoint(const proto::Path &path, const geometry2::Vec2f current_pose,
                                       PathPointIter *nearest_point_iter,
                                       float *min_squared_dist, int *index) {
        if (nearest_point_iter == nullptr || min_squared_dist == nullptr) {
            return false;
        }
        if (!path.path_points().empty()) {
            *min_squared_dist = squaredDistance(
                current_pose, geometry2::Segment{path.path_points().begin()->position(),
                                                 (path.path_points().begin() + 1)->position()});
            PathPointIter iter, end_iter, nearset_iter;
            iter = path.path_points().begin();
            nearset_iter = iter;
            *index = 0;
            // end_iter =  ((path.path_points().begin() )<= (path.path_points().end() - 1)) ? (path.path_points().begin() + (*index) + 200) : (path.path_points().end() - 1);
            for (iter = path.path_points().begin(); iter < path.path_points().end() - 1; iter++) {
                float new_squared_dist = squaredDistance(current_pose,
                                                         geometry2::Segment{iter->position(), (iter + 1)->position()});
                if (new_squared_dist < *min_squared_dist) {
                    nearset_iter = iter;
                    *min_squared_dist = new_squared_dist;
                    *index = static_cast<int>(std::distance(path.path_points().begin(), iter));
                }
            }
            // if the car pose is far away from the path.
            if (*min_squared_dist > point_on_path_threshold_) {
                ROS_INFO("The robot pose is far away from the path.");
                return false;
            }

            *nearest_point_iter = nearset_iter;
            return true;
        }
        return false;
    }

    bool PathMatcher::findInCiclePoint(const proto::Path &path, const geometry2::Vec2f current_pose, PathPointIter *nearest_point_iter,
                                       const float dis, const int current_index) {
        if (nearest_point_iter == nullptr) {
            return false;
        }
        float min_squared_dist = 0.0f;
        if (!path.path_points().empty()) {
            PathPointIter begin_iter = path.path_points().begin();
            begin_iter = ((path.path_points().begin() + current_index )<= (path.path_points().end() - 1)) ? (path.path_points().begin() + current_index) : (path.path_points().end() - 1);
            min_squared_dist = squaredDistance(
                                current_pose, geometry2::Segment{begin_iter->position(),
                                (begin_iter + 1)->position()});

            PathPointIter iter, pre_iter = path.path_points().begin(), end_iter;

            end_iter =  ((path.path_points().begin() + current_index + 30 )<= (path.path_points().end() - 1)) ? (path.path_points().begin() + current_index + 30) : (path.path_points().end() - 1);
            for (iter = path.path_points().begin() + current_index; iter < end_iter; iter++) {
                float new_squared_dist = squaredDistance(current_pose,
                                     geometry2::Segment{iter->position(), (iter + 1)->position()});
                if (new_squared_dist > dis || new_squared_dist > point_on_path_threshold_) {
                    continue;
                } else {
                    pre_iter = iter;
                    min_squared_dist = new_squared_dist;
                }
            }
            // if the car pose is far away from the path.
            if (min_squared_dist > point_on_path_threshold_) {
                ROS_INFO("The robot pose is far away from the path.");
                return false;
            }
            *nearest_point_iter = pre_iter;
            return true;
        }
        return false;

    }

    bool PathMatcher::findNearestPoint(const proto::Path &path, const geometry2::Vec2f current_pose,
                                       PathPointIter *nearest_point_iter, int *index) {
        float min_squared_dist = 0.0;
        return findNearestPoint(path, current_pose, nearest_point_iter, &min_squared_dist, index);
    }

    bool PathMatcher::updateMatchedPoints(const proto::Path &path, const proto::RobotState &robot_state,
                                          float look_ahead_dist, bool reverse, geometry2::Vec2f &anchor_point_ref,
                                          int *index, const float box_size, const float iter_num,
                                          proto::AnchorBox& anchor_boxes, bool& should_rotate, int& rotate_index) {
        float theta = robot_state.roll_pitch_yaw().z();
        // if robot wanna back up
        if (reverse) {
            theta = robot_state.roll_pitch_yaw().z() - M_PI;
        }
        const geometry2::Vec2f current_pose = geometry2::Vec2f(robot_state.position());

        PathPointIter iter, box_iter;

        if (findInCiclePoint(path, current_pose, &iter, 0.05, *index)) {
            matched_points_iter_.rear_axis_iter = iter;
            if (iter == path.path_points().end() - 1) {
                // the nearest point of rear axis on path already reach the end of path
                ROS_INFO("Robot rear axis is at the end of path.");
                return false;
            }
        } else {
            ROS_WARN("Cannot find nearset point of robot rear axis on path.");
            return false;
        }
        proto::Pose* temp_pose;
        temp_pose = anchor_boxes.add_anchor_pose();
        temp_pose->mutable_pose()->set_x(iter->position().x());
        temp_pose->mutable_pose()->set_y(iter->position().y());
        temp_pose->mutable_pose()->set_z(iter->theta());
        geometry_msgs::Quaternion qua = tf::createQuaternionMsgFromYaw((iter->theta()));

        temp_pose->mutable_qua()->set_x(qua.x);
        temp_pose->mutable_qua()->set_y(qua.y);
        temp_pose->mutable_qua()->set_z(qua.z);
        temp_pose->mutable_qua()->set_w(qua.w);

        temp_pose->set_index(iter - path.path_points().begin());

        int it_num = 0;
        for (int i = 1; i < box_size; ++i) {
            if (iter_num * i > (path.path_points().end() - 1 - iter)) {
                it_num = path.path_points().end() - 1 - iter;
            } else {
                it_num = iter_num * i - 1;

            }
            if (path.path_points().end() - iter > it_num) {
                box_iter = iter + it_num;
            } else {
                box_iter = path.path_points().end() - 1;
            }

            proto::Pose* path_point;
            path_point = anchor_boxes.add_anchor_pose();
            path_point->mutable_pose()->set_x(box_iter->position().x());
            path_point->mutable_pose()->set_y(box_iter->position().y());
            path_point->mutable_pose()->set_z(box_iter->theta());

            geometry_msgs::Quaternion qua = tf::createQuaternionMsgFromYaw(box_iter->theta());
            path_point->mutable_qua()->set_x(qua.x);
            path_point->mutable_qua()->set_y(qua.y);
            path_point->mutable_qua()->set_z(qua.z);
            path_point->mutable_qua()->set_w(qua.w);

            path_point->set_index(box_iter - path.path_points().begin());
        }
        if (path.path_points().end() - iter > static_cast<int>(look_ahead_dist)) {
            matched_points_iter_.fb_point_iter = iter + static_cast<int>(look_ahead_dist) - 1;
        } else {
            matched_points_iter_.fb_point_iter = path.path_points().end();
        }
        geometry2::Vec2f fb_vec = geometry2::Vec2f((matched_points_iter_.fb_point_iter - 1)->position()) - current_pose;
        steer_error_.lon = fb_vec.x * cos(theta) + fb_vec.y * sin(theta);
        steer_error_.lat = -fb_vec.x * sin(theta) + fb_vec.y * cos(theta);
        steer_error_.theta = theta - iter->theta();

        anchor_point_ref.x = (matched_points_iter_.fb_point_iter - 1)->position().x();
        anchor_point_ref.y = (matched_points_iter_.fb_point_iter - 1)->position().y();
        // if (matched_points_iter_.fb_point_iter == path.path_points().end()) {
        //    // Expand 0.7m
        //     anchor_point_ref.x += 0.7 * cos(matched_points_iter_.fb_point_iter->theta());
        //     anchor_point_ref.y += 0.7 * sin(matched_points_iter_.fb_point_iter->theta());
        // }


        MatchedPointsIter rotate_points_iter;
        if (path.path_points().end() - iter > 2 *static_cast<int>(look_ahead_dist)) {
            rotate_points_iter.fb_point_iter = iter + 2 * static_cast<int>(look_ahead_dist) - 1;
        } else {
            rotate_points_iter.fb_point_iter = path.path_points().end();
        }
        geometry2::Vec2f anchor_point_1, anchor_point_2;

        if (rotate_points_iter.fb_point_iter != path.path_points().end()) {
            anchor_point_1 = geometry2::Vec2f((rotate_points_iter.fb_point_iter - 1)->position());
            anchor_point_2 = geometry2::Vec2f((rotate_points_iter.fb_point_iter)->position());
        } else {
            anchor_point_1 = geometry2::Vec2f((rotate_points_iter.fb_point_iter - 2)->position());
            anchor_point_2 = geometry2::Vec2f((rotate_points_iter.fb_point_iter - 1)->position());
        }

        float cur_anchor1= (anchor_point_1 - current_pose).heading();
        float anchor1_anchor2 = (anchor_point_2 - anchor_point_1).heading();

        float deta_heading = cur_anchor1 - anchor1_anchor2;
        // robot need rotate if curverture is too large
        // if (fabs(deta_heading) >  M_PI/ 4) {
        //     should_rotate = true;
        // } else {
        //     //should_rotate = false;
        // }
        rotate_index = rotate_points_iter.fb_point_iter -  path.path_points().begin();
        return true;
    }

    float PathMatcher::getRobotTargetDistSquare(const geometry2::Vec2f current_pose) const {
        geometry2::Vec2f preview =
                geometry2::Vec2f((matched_points_iter_.fb_point_iter - 1)->position()) - current_pose;

        return preview.dot(preview);
    }
} //control
