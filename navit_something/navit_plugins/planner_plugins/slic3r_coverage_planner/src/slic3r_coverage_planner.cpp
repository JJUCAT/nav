#include <slic3r_coverage_planner/slic3r_coverage_planner.h>

namespace slic3r_coverage_planner
{
  void Slic3rCoveragePlanner::initialize(const std::string& name) 
  {
      ros::NodeHandle pnh("~" + name);

      pnh.param("outer_offset", outer_offset_, 0.5);
      pnh.param("distance", distance_, 0.5);
      pnh.param("is_contour", is_contour_, true);
      pnh.param("angle", angle_, 0.0);
    
  }

  bool Slic3rCoveragePlanner::makePlan(const geometry_msgs::Polygon& coverage_area,
                                       const std::vector<geometry_msgs::Polygon>& holes,
                                       const geometry_msgs::Pose& start,
                                       const geometry_msgs::Pose& end,
                                       std::vector<nav_msgs::Path>& coverage_path,
                                       std::vector<nav_msgs::Path>& contour_path)
  {
      // sanity check
      if (coverage_area.points.size() < 3) {
          ROS_ERROR("Coverage area must have at least 3 points.");
          return false;
        }

      // TODO(薛漫天): check if holes coordinates are valid, they should be inside the coverage area
      

        Slic3r::Polygon outline_poly;
        
        for (auto &pt: coverage_area.points) {
            outline_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
        }

        outline_poly.make_counter_clockwise();

        // This ExPolygon contains our input area with holes.
        Slic3r::ExPolygon expoly(outline_poly);

        for (auto &hole: holes) {
            Slic3r::Polygon hole_poly;
            for (auto &pt: hole.points) {
                hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
            }
        
            hole_poly.make_clockwise();

            expoly.holes.push_back(hole_poly);
        }


        // Results are stored here
        std::vector<Polygons> area_outlines;
        Polylines fill_lines;
        std::vector<Polygons> obstacle_outlines;


        coord_t distance = scale_(distance_);
        coord_t outer_distance = scale_(outer_offset_);

        // detect how many perimeters must be generated for this island
        int loops = 1;

        ROS_INFO_STREAM("generating " << loops << " outlines");

        int outline_overlap_count = 0;
        const int loop_number = loops - 1;  // 0-indexed loops
        const int inner_loop_number = loop_number - outline_overlap_count;

        Polygons gaps;
        Polygons last = expoly;
        Polygons inner = last;
        
        if (loop_number >= 0) 
        {  // no loops = -1

            std::vector<PerimeterGeneratorLoops> contours(loop_number + 1);    // depth => loops
            std::vector<PerimeterGeneratorLoops> holes(loop_number + 1);       // depth => loops

            for (int i = 0; i <= loop_number; ++i) {  // outer loop is 0
                Polygons offsets;

                if (i == 0) {
                    offsets = offset( last, -outer_distance);

                } else {
                
                    offsets = offset( last, -distance);
                }

                if (offsets.empty()) break;

                last = offsets;
            
                if(i <= inner_loop_number) {
                    inner = last;
                }

                for (Polygons::const_iterator polygon = offsets.begin(); polygon != offsets.end(); ++polygon) {
                    PerimeterGeneratorLoop loop(*polygon, i);
                    loop.is_contour = polygon->is_counter_clockwise();
                    
                    if (loop.is_contour) {
                        contours[i].push_back(loop);
                    } else {
                        holes[i].push_back(loop);
                    }
                }
            }

            // nest loops: holes first
            for (int d = 0; d <= loop_number; ++d) {
                PerimeterGeneratorLoops &holes_d = holes[d];

                // loop through all holes having depth == d
                for (int i = 0; i < (int) holes_d.size(); ++i) {
                    const PerimeterGeneratorLoop &loop = holes_d[i];

                    // find the hole loop that contains this one, if any
                    for (int t = d + 1; t <= loop_number; ++t) {
                        for (int j = 0; j < (int) holes[t].size(); ++j) {
                            PerimeterGeneratorLoop &candidate_parent = holes[t][j];
                            if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                                candidate_parent.children.push_back(loop);
                                holes_d.erase(holes_d.begin() + i);
                                --i;
                                goto NEXT_LOOP;
                            }
                        }
                    }

                    NEXT_LOOP:;
                }
            }

        // nest contour loops
        for (int d = loop_number; d >= 1; --d) 
        {
            PerimeterGeneratorLoops &contours_d = contours[d];

            // loop through all contours having depth == d
            for (int i = 0; i < (int) contours_d.size(); ++i) {
                const PerimeterGeneratorLoop &loop = contours_d[i];

                // find the contour loop that contains it
                for (int t = d - 1; t >= 0; --t) {
                    for (size_t j = 0; j < contours[t].size(); ++j) {
                        PerimeterGeneratorLoop &candidate_parent = contours[t][j];
                        if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                            candidate_parent.children.push_back(loop);
                            contours_d.erase(contours_d.begin() + i);
                            --i;
                            goto NEXT_CONTOUR;
                        }
                    }
                }
                NEXT_CONTOUR:;
            }
        }

        traverse(contours[0], area_outlines);
        for(auto &hole:holes) {
            traverse(hole, obstacle_outlines);
        }

        for(auto &obstacle_group : obstacle_outlines) {
            std::reverse(obstacle_group.begin(), obstacle_group.end());
        }

    }// end of if (loop_number >= 0)

        ExPolygons expp = union_ex(inner);

        // Go through the innermost poly and create the fill path using a Fill object
        for (auto &poly: expp) {
            Slic3r::Surface surface(Slic3r::SurfaceType::stBottom, poly);

            Slic3r::Fill *fill;
            if (is_contour_) {
                fill = new Slic3r::FillConcentric();
            }
            else {
                fill = new Slic3r::FillRectilinear();
            }
            fill->link_max_length = scale_(1.0);
            fill->angle = angle_;
            fill->z = scale_(1.0);
            fill->endpoints_overlap = 0;
            fill->density = 1.0;
            fill->dont_connect = false;
            fill->dont_adjust = false;
            fill->min_spacing = distance_;
            fill->complete = false;
            fill->link_max_length = 0;

            ROS_INFO_STREAM("Starting Fill. Poly size:" << surface.expolygon.contour.points.size());

            Slic3r::Polylines lines = fill->fill_surface(surface);
            append_to(fill_lines, lines);
            delete fill;
            fill = nullptr;

            ROS_INFO_STREAM("Fill Complete. Polyline count: " << lines.size());
            for (int i = 0; i < lines.size(); i++) {
                ROS_INFO_STREAM("Polyline " << i << " has point count: " << lines[i].points.size());
            }
        }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    header.seq = 0;

    // contour paths
    for(auto &group:area_outlines) {
        nav_msgs::Path path;
        path.header = header;
        int split_index = 0;
        for (int i = 0; i < group.size(); i++) {
            auto &poly = group[i];

            Polyline line;
            if(split_index < poly.points.size()) {
                line = poly.split_at_index(split_index);
            } else {
                line = poly.split_at_first_point();
                split_index = 0;
            }
            split_index+=2;
            line.remove_duplicate_points();

            auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
            if (equally_spaced_points.size() < 2) {
                ROS_INFO("Skipping single dot");
                continue;
            }
            ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

            Point *lastPoint = nullptr;
            for (auto &pt: equally_spaced_points) {
                if (lastPoint == nullptr) {
                    lastPoint = &pt;
                    continue;
                }

                // calculate pose for "lastPoint" pointing to current point

                auto dir = pt - *lastPoint;
                double orientation = atan2(dir.y, dir.x);
                tf2::Quaternion q(0.0, 0.0, orientation);

                geometry_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.orientation = tf2::toMsg(q);
                pose.pose.position.x = unscale(lastPoint->x);
                pose.pose.position.y = unscale(lastPoint->y);
                pose.pose.position.z = 0;
                path.poses.push_back(pose);
                lastPoint = &pt;
            }

            // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.poses.push_back(pose);

        }
        contour_path.push_back(path);
    }

    for (int i = 0; i < fill_lines.size(); i++) {
        auto &line = fill_lines[i];
        nav_msgs::Path cover_path;
        cover_path.header = header;

        line.remove_duplicate_points();


        auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
        if (equally_spaced_points.size() < 2) {
            ROS_INFO("Skipping single dot");
            continue;
        }
        ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

        Point *lastPoint = nullptr;
        for (auto &pt: equally_spaced_points) {
            if (lastPoint == nullptr) {
                lastPoint = &pt;
                continue;
            }

            // calculate pose for "lastPoint" pointing to current point

            auto dir = pt - *lastPoint;
            double orientation = atan2(dir.y, dir.x);
            tf2::Quaternion q(0.0, 0.0, orientation);

            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = tf2::toMsg(q);
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            cover_path.poses.push_back(pose);
            lastPoint = &pt;
        }

        // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose.orientation = cover_path.poses.back().pose.orientation;
        pose.pose.position.x = unscale(lastPoint->x);
        pose.pose.position.y = unscale(lastPoint->y);
        pose.pose.position.z = 0;
        cover_path.poses.push_back(pose);

        coverage_path.push_back(cover_path);
    }

    for(auto &group:obstacle_outlines) {
        nav_msgs::Path obs_path;
        obs_path.header = header;
        int split_index = 0;
        for (int i = 0; i < group.size(); i++) {
            auto &poly = group[i];

            Polyline line;
            if(split_index < poly.points.size()) {
                line = poly.split_at_index(split_index);
            } else {
                line = poly.split_at_first_point();
                split_index = 0;
            }
            split_index+=2;
            line.remove_duplicate_points();

            auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
            if (equally_spaced_points.size() < 2) {
                ROS_INFO("Skipping single dot");
                continue;
            }
            ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

            Point *lastPoint = nullptr;
            for (auto &pt: equally_spaced_points) {
                if (lastPoint == nullptr) {
                    lastPoint = &pt;
                    continue;
                }

                // calculate pose for "lastPoint" pointing to current point

                auto dir = pt - *lastPoint;
                double orientation = atan2(dir.y, dir.x);
                tf2::Quaternion q(0.0, 0.0, orientation);

                geometry_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.orientation = tf2::toMsg(q);
                pose.pose.position.x = unscale(lastPoint->x);
                pose.pose.position.y = unscale(lastPoint->y);
                pose.pose.position.z = 0;
                obs_path.poses.push_back(pose);
                lastPoint = &pt;
            }

            // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = obs_path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            obs_path.poses.push_back(pose);

        }
        contour_path.push_back(obs_path);
    }

    // check if coverage path is empty
    if (coverage_path.empty()) {
        ROS_WARN("Coverage path is empty");
        return false;
    }


    return true;
    
  }
}// namespace slic3r_coverage_planner
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(slic3r_coverage_planner::Slic3rCoveragePlanner, navit_core::CoveragePlanner)
