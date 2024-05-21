#ifndef _LINE_H
#define _LINE_H

/**
 * \file Line.h
 *
 * \brief This file implements a parametrized line.
 */

#include "Ar.h"
#include "Util.h"

namespace dock_visual_perception
{
/**
 * \brief Struct representing a line. The line is parametrized by its center and
 * direction vector.
 */
struct AR_EXPORT Line
{
  /**
   * \brief Constructor.
   * \param params params[0] and params[1] are the x and y components of the
   * direction vector, params[2] and params[3] are the x and y coordinates of
   * the line center.
   */
  Line(const cv::Vec4f& params);
  Line()
  {
  }

  /**
   * \brief Line center.
   */
  PointDouble c;  // center
  /**
   * \brief Direction vector.
   */
  PointDouble s;  // direction vector
};

/**
 * \brief Fit lines to vector of points.
 * \param lines		Resulting set of lines.
 * \param corners	Index list of line breaks.
 * \param edge		Vector of points (pixels) where the line is fitted.
 * \param grey		In the future, we may want to fit lines directly to grayscale
 * image instead of thresholded edge.
 */
int AR_EXPORT FitLines(std::vector<Line>& lines,
                          const std::vector<int>& corners,
                          const std::vector<PointInt>& edge);

/**
 * \brief Calculates an intersection point of two lines.
 * \param l1	First line.
 * \param l2	Second line.
 * \return		Intersection point.
 */
PointDouble AR_EXPORT Intersection(const Line& l1, const Line& l2);

}

#endif
