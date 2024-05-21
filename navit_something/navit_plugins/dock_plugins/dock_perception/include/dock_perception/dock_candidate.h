#ifndef DOCK_CANDIDATE_H
#define DOCK_CANDIDATE_H

#include <geometry_msgs/Point.h>

#include <vector>

/**
 * @brief A cluster which is a candidate for a dock
 */
struct DockCandidate {
  std::vector<geometry_msgs::Point> points;
  double dist; // distance from initial/previous pose

  /** @brief Get the width of this segment */
  double width() {
    // If there are no points then there is no width.
    if (points.empty()) {
      return 0;
    }

    geometry_msgs::Point &pt1 = points.front();
    geometry_msgs::Point &pt2 = points.back();
    return (sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2)));
  }

  /**
   * @brief Determine if this candidate meets our basic criteria
   * @param dock_found Has the dock been found in a previous frame?
   * @TODO(Max): replace magic numbers with DockShape attributes
   */
  bool valid(bool dock_found) {
    // If there are no points this cannot be valid.
    if (points.empty()) {
      return false;
    }

    // Check overall size
    if (width() > 1.5 || width() < 0.15)
      return false;

    // If dock is found, we want to avoid large jumps
    if (dock_found) {
      // dist is squared
      return dist < (0.25 * 0.25);
    }

    // Not too far off from initial pose
    return dist < 1.0;
  }
};
typedef std::shared_ptr<DockCandidate> DockCandidatePtr;

struct CompareCandidates {
  bool operator()(DockCandidatePtr a, DockCandidatePtr b) {
    return (a->dist > b->dist);
  }
};

#endif // DOCK_CANDIDATE_H
