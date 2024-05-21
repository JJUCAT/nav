/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for vanjee data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#ifndef __vanjee_pointcloud_POINT_TYPES_H
#define __vanjee_pointcloud_POINT_TYPES_H

#include <pcl/point_types.h>

namespace vanjee_lidar
{
  /** Euclidean vanjee coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    double timestamp;                    // lidar time
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace vanjee_lidar


POINT_CLOUD_REGISTER_POINT_STRUCT(vanjee_lidar::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (double ,timestamp, timestamp)
                                  )
                                  
#endif // __vanjee_pointcloud_POINT_TYPES_H

