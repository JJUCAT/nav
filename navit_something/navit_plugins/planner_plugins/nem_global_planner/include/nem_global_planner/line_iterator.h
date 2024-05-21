/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef LINE_ITERATOR_H
#define LINE_ITERATOR_H

#include <stdlib.h>

namespace nem_global_planner {

/** An iterator implementing Bresenham Ray-Tracing by resolution of distance.*/
class LineIterator {
   public:
    LineIterator(double x0, double y0, double x1, double y1, double res)
        : x0_(x0),
          y0_(y0),
          x1_(x1),
          y1_(y1),
          x_(x0),  // X and Y start of at first endpoint.
          y_(y0),
          deltax_(abs(x1 - x0)),
          deltay_(abs(y1 - y0)),
          curpixel_(0) {
        if (deltax_ == 0.0) {
            res_x_ = 0.0;
            res_y_ = res;
        } else {
            angle_ = atan((y1_ - y0_) / (x1_ - x0_));
            res_x_ = fabs(res * cos(angle_));
            res_y_ = fabs(res * sin(angle_));
        }

        if (x1_ >= x0_)  // The x-values are increasing
        {
            xinc1_ = res_x_;
            // xinc2_ = res_x_;
        } else  // The x-values are decreasing
        {
            xinc1_ = -res_x_;
            // xinc2_ = -res_x_;
        }

        if (y1_ >= y0_)  // The y-values are increasing
        {
            yinc1_ = res_y_;
            // yinc2_ = res_y_;
        } else  // The y-values are decreasing
        {
            yinc1_ = -res_y_;
            // yinc2_ = -res_y_;
        }

        if (deltax_ >= deltay_)  // There is at least one x-value for every y-value
        {
            //                xinc1_ = 0;                  // Don't change the x when numerator >= denominator
            //                yinc2_ = 0;                  // Don't change the y for every iteration
            //                den_ = deltax_;
            //                num_ = deltax_ / 2;
            //                numadd_ = deltay_;
            numpixels_ = deltax_;  // There are more x-values than y-values
        } else                     // There is at least one y-value for every x-value
        {
            //                xinc2_ = 0;                  // Don't change the x for every iteration
            //                yinc1_ = 0;                  // Don't change the y when numerator >= denominator
            //                den_ = deltay_;
            //                num_ = deltay_ / 2;
            //                numadd_ = deltax_;
            numpixels_ = deltay_;  // There are more y-values than x-values
        }
    }

    bool isValid() const { return curpixel_ <= numpixels_; }

    void advance() {
        //            num_ += numadd_;              // Increase the numerator by the top of the fraction
        //            if (num_ >= den_)            // Check if numerator >= denominator
        //            {
        //                num_ -= den_;               // Calculate the new numerator value
        //                x_ += xinc1_;               // Change the x as appropriate
        //                y_ += yinc1_;               // Change the y as appropriate
        //            }
        x_ += xinc1_;  // Change the x as appropriate
        y_ += yinc1_;  // Change the y as appropriate

        if (deltax_ >= deltay_)
            curpixel_ += res_x_;
        else
            curpixel_ += res_y_;
    }

    double getX() const { return x_; }

    double getY() const { return y_; }

    double getX0() const { return x0_; }

    double getY0() const { return y0_; }

    double getX1() const { return x1_; }

    double getY1() const { return y1_; }

   private:
    double x0_;  ///< X coordinate of first end point.
    double y0_;  ///< Y coordinate of first end point.
    double x1_;  ///< X coordinate of second end point.
    double y1_;  ///< Y coordinate of second end point.

    double x_;  ///< X coordinate of current point.
    double y_;  ///< Y coordinate of current point.

    double deltax_;  ///< Difference between Xs of endpoints.
    double deltay_;  ///< Difference between Ys of endpoints.

    double curpixel_;  ///< index of current point in line loop.

    double xinc1_, xinc2_, yinc1_, yinc2_;
    double den_, num_, numadd_, numpixels_;

    double angle_, res_x_, res_y_;
};

}  // end namespace node_edge_path_plan

#endif  // LINE_ITERATOR_H
