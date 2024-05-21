/*
* @Author: czk
* @Date:   2022-10-02 12:13:25
* @Last Modified by:   chenzongkui
* @Last Modified time: 2023-03-21 10:36:26
*/
#include "path_smoother/path_spline.h"


namespace navit_planner {

bool PathGslSpline::pathSmoother(const nav_msgs::Path& rough_path, nav_msgs::Path& smooth_path) {
    proto::Polyline rough_path_proto;

    for (int i = 0; i < rough_path.poses.size(); ++i) {
        proto::Vector3f *v3f;
        v3f = rough_path_proto.add_points();
        v3f->set_x(rough_path.poses[i].pose.position.x);
        v3f->set_y(rough_path.poses[i].pose.position.y);
    }

    proto::Path smooth_path_proto;
    proto::Path *smooth_path_proto_ptr = &smooth_path_proto;

    smooth_path.header.frame_id = rough_path.header.frame_id;

    if (setPath(rough_path_proto, smooth_path_proto_ptr)) {
        for (int i = 0; i < smooth_path_proto_ptr->path_points().size(); ++i) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = smooth_path_proto_ptr->path_points(i).position().x();
            pose.pose.position.y = smooth_path_proto_ptr->path_points(i).position().y();
            smooth_path.poses.push_back(pose);
        }
        return true;
    } else {
        return false;
    }
}

bool PathGslSpline::pathSmoother(const nav_msgs::Path& rough_path, proto::Path& smooth_path) {
    proto::Polyline rough_path_proto;

    for (int i = 0; i < rough_path.poses.size(); ++i) {
        proto::Vector3f *v3f;
        v3f = rough_path_proto.add_points();
        v3f->set_x(rough_path.poses[i].pose.position.x);
        v3f->set_y(rough_path.poses[i].pose.position.y);
        v3f->set_z(rough_path.poses[i].pose.position.z);
    }
    proto::Path smooth_path_proto_ptr;
    if (setPath(rough_path_proto, &smooth_path_proto_ptr)) {
        for (int i = 0; i < smooth_path_proto_ptr.path_points().size(); ++i) {
            proto::PathPoint* path_point;
            path_point = smooth_path.add_path_points();
            path_point->mutable_position()->set_x(smooth_path_proto_ptr.path_points(i).position().x());
            path_point->mutable_position()->set_y(smooth_path_proto_ptr.path_points(i).position().y());
            path_point->set_curvature(smooth_path_proto_ptr.path_points(i).curvature());
            if (i >= 1) {
                path_point->set_theta(normalizeAngle(atan2((smooth_path_proto_ptr.path_points(i).position().y() - smooth_path_proto_ptr.path_points(i-1).position().y())
                                           ,(smooth_path_proto_ptr.path_points(i).position().x() - smooth_path_proto_ptr.path_points(i-1).position().x()))));
            } else {
                path_point->set_theta(normalizeAngle(atan2((smooth_path_proto_ptr.path_points(1).position().y() - smooth_path_proto_ptr.path_points(0).position().y())
                                           ,(smooth_path_proto_ptr.path_points(1).position().x() - smooth_path_proto_ptr.path_points(0).position().x()))));
            }

            float vel;
            vel = 1.0 / (5.0 * fabs(smooth_path_proto_ptr.path_points(i).curvature()) + 1.0f);

            // static float last_vel = 0;
            // static float vel_buf[20] = {0};
            // static int buf_index = 0;
            // vel_buf[buf_index++] = vel;
            // if (buf_index == 20) {
            //     buf_index = 0;
            // }
            // float sum_vel = 0;
            // for (int j = 0; j < 20; j++) {
            //     sum_vel += vel_buf[j];
            // }
            // vel = sum_vel / 20;
            // last_vel = vel;

            // path_point->set_speed(vel);
            path_point->set_s(smooth_path_proto_ptr.path_points(i).s());

            int lookahead_distance = 150;
            int lookbehind_distance = 50;
            float high_curvature_threshold = 0.6;
            float min_vel = 0.2;
            float deceleration_rate = 0.03;

            for (int j = i + 1; j <= std::min(i + lookahead_distance, smooth_path_proto_ptr.path_points().size() - 1); ++j) {
                if (fabs(smooth_path_proto_ptr.path_points(j).curvature()) > high_curvature_threshold) {
                    float distance_to_curve = j - i;
                    float deceleration = deceleration_rate * distance_to_curve;
                    vel = std::max(min_vel, vel - deceleration);
                    break;
                }
            }

            for (int j = i - 1; j >= std::max(i - lookbehind_distance, 0); --j) {
                if (fabs(smooth_path_proto_ptr.path_points(j).curvature()) > high_curvature_threshold) {
                    float distance_to_curve = i - j;
                    float deceleration = deceleration_rate * distance_to_curve;
                    vel = std::max(min_vel, vel - deceleration);
                    break;
                }
            }

            vel = (vel < 0.2 ?  0.2: vel);
            vel = std::min(vel, smooth_path_proto_ptr.path_points(i).speed_limit());
            path_point->set_speed(vel);
        }

        // 对到点轨迹进行了平滑
        float smooth_vel_dis = 2.5;
        for (int i = 0; i < smooth_path_proto_ptr.path_points().size(); ++i) {
            if (smooth_path_proto_ptr.path_points(i).s() < smooth_vel_dis) {
                float velocity = smooth_path_proto_ptr.path_points(i).s() / smooth_vel_dis;
                smooth_path_proto_ptr.mutable_path_points(i)->set_speed(std::min(velocity, smooth_path_proto_ptr.path_points(i).speed()));
            }
            else if (smooth_path_proto_ptr.path_points(i).s() > smooth_path_proto_ptr.path_points(smooth_path_proto_ptr.path_points().size() - 1).s() - smooth_vel_dis) {
                float velocity = 1.0 - (smooth_path_proto_ptr.path_points(i).s() - (smooth_path_proto_ptr.path_points(smooth_path_proto_ptr.path_points().size() - 1).s() - smooth_vel_dis)) / smooth_vel_dis;
                smooth_path_proto_ptr.mutable_path_points(i)->set_speed(std::min(velocity, smooth_path_proto_ptr.path_points(i).speed()));
            }
        }
        return true;
    } else {
        ROS_ERROR("Set path failed.");
        return false;
    }
}
bool PathGslSpline::pathSmoother(const proto::Path& rough_path, proto::Path& smooth_path) {
    proto::Polyline rough_path_proto;

    for (int i = 0; i < rough_path.path_points_size(); ++i) {
        proto::Vector3f *v3f;
        v3f = rough_path_proto.add_points();
        v3f->set_x(rough_path.path_points(i).position().x());
        v3f->set_y(rough_path.path_points(i).position().y());
        v3f->set_z(rough_path.path_points(i).position().z());

    }
    proto::Path smooth_path_proto_ptr;
    if (setPath(rough_path_proto, &smooth_path_proto_ptr)) {
        for (int i = 0; i < smooth_path_proto_ptr.path_points().size(); ++i) {
            proto::PathPoint* path_point;
            path_point = smooth_path.add_path_points();
            path_point->mutable_position()->set_x(smooth_path_proto_ptr.path_points(i).position().x());
            path_point->mutable_position()->set_y(smooth_path_proto_ptr.path_points(i).position().y());
            path_point->set_curvature(smooth_path_proto_ptr.path_points(i).curvature());
            if (i >= 1) {
                path_point->set_theta(normalizeAngle(atan2((smooth_path_proto_ptr.path_points(i).position().y() - smooth_path_proto_ptr.path_points(i-1).position().y())
                                           ,(smooth_path_proto_ptr.path_points(i).position().x() - smooth_path_proto_ptr.path_points(i-1).position().x()))));
            } else {
                path_point->set_theta(normalizeAngle(atan2((smooth_path_proto_ptr.path_points(1).position().y() - smooth_path_proto_ptr.path_points(0).position().y())
                                           ,(smooth_path_proto_ptr.path_points(1).position().x() - smooth_path_proto_ptr.path_points(0).position().x()))));
            }

            float vel;
            path_point->set_speed(vel > path_point->speed() ? path_point->speed() : vel );
            path_point->set_s(smooth_path_proto_ptr.path_points(i).s());

            int lookahead_distance = 80;
            float high_curvature_threshold = 0.3;
            float min_vel = 0.2;
            float deceleration_rate = 0.01;

            for (int j = i + 1; j <= std::min(i + lookahead_distance, smooth_path_proto_ptr.path_points().size() - 1); ++j) {
                if (fabs(smooth_path_proto_ptr.path_points(j).curvature()) > high_curvature_threshold) {
                    float distance_to_curve = j - i;
                    float deceleration = deceleration_rate * distance_to_curve;
                    vel = std::max(min_vel, vel - deceleration);
                    break;
                }
            }
            vel = (vel < 0.2 ?  0.2: vel);

            path_point->set_speed(vel > smooth_path_proto_ptr.path_points(i).speed() ? smooth_path_proto_ptr.path_points(i).speed()
                                                                                : vel);
        }
        return true;
    } else {
        ROS_ERROR("Set path failed.");
        return false;
    }
}
    bool PathGslSpline::setPath(const proto::Polyline& line, proto::Path* path) {
        if (line.points_size() < 2) return false;
        std::vector<double> x, y, s, k;
        proto::Polyline polyline;

        if (line.points_size() < step_point_ - 1) {
            return false;
        }
        int j = 0;
        for (int i = 0; i < line.points_size() - 1; i = i + step_point_) {
                j = j + step_point_;
                x.push_back(line.points(i).x());
                y.push_back(line.points(i).y());
                s.push_back(i == 0 ? 0.0 : s.back() + hypotFast(line.points(i).x() - line.points(i - step_point_).x(),
                                                                line.points(i).y() - line.points(i - step_point_).y()));
                k.push_back(line.points(i).z());
                j = i;
        }
        x.push_back(line.points(line.points_size() - 1).x());
        y.push_back(line.points(line.points_size() - 1).y());

        s.push_back(s.back() + hypotFast(line.points(line.points_size() - 1).x() - line.points(j).x(),
                                        line.points(line.points_size() - 1).y() - line.points(j).y()));

        spline_x_ = gsl_spline_alloc(gsl_interp_cspline, x.size());
        spline_y_ = gsl_spline_alloc(gsl_interp_cspline, y.size());

        gsl_spline_init(spline_x_, s.data(), x.data(), x.size());
        gsl_spline_init(spline_y_, s.data(), y.data(), y.size());

        proto::PathPoint path_point;
        float length = s.back();

        for (float s = 0.0; s <= length; s += step_s_) {
            if (!evaluate(s, &path_point)) break;
            int k_index = static_cast<int>(s / (length / k.size()));
            if (k[k_index] > 0.01) {
                path_point.set_speed_limit(k[k_index]);
            } else {
                path_point.set_speed_limit(1.0);
            }

            proto::PathPoint* path_point_ptr = path->add_path_points();
            *path_point_ptr = path_point;
        }

        gsl_spline_free(spline_x_);
        gsl_spline_free(spline_y_);
        gsl_interp_accel_free(acc_);

        return true;
    }

bool PathGslSpline::evaluate(double s, proto::PathPoint* path_point) {
  double dx;
  acc_ = gsl_interp_accel_alloc();
  if (gsl_spline_eval_deriv_e(spline_x_, s, acc_, &dx) != 0) {
    return false;
  }

  double dy = gsl_spline_eval_deriv(spline_y_, s, acc_);
  double ddx = gsl_spline_eval_deriv2(spline_x_, s, acc_);
  double ddy = gsl_spline_eval_deriv2(spline_y_, s, acc_);

  path_point->mutable_position()->set_x(gsl_spline_eval(spline_x_, s, acc_));
  path_point->mutable_position()->set_y(gsl_spline_eval(spline_y_, s, acc_));
  path_point->mutable_position()->set_z(0.0);
  path_point->set_theta(std::atan2(dy, dx));
  path_point->set_curvature((dx * ddy - dy * ddx) / cubic(hypotFast(dx, dy)));
  path_point->set_s(s);
  path_point->set_l(0.0);
  return true;
}
}  // namespace navit_planner
PLUGINLIB_EXPORT_CLASS(navit_planner::PathGslSpline, navit_core::SmoothPathPlanner)