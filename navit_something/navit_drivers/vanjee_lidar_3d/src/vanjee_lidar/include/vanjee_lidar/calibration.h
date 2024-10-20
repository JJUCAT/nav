/**
 * \file  calibration.h 
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology, University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:25:34 AM piyushk $
 */

#ifndef __VANJEE_CALIBRATION_H
#define __VANJEE_CALIBRATION_H

#include <map>
#include <string>

namespace vanjee_lidar {

/** \brief Correction information for a single laser. */
struct LaserCorrection {

	/** parameters in db.xml */
	float rot_correction;
	float vert_correction;
	float dist_correction;
	bool two_pt_correction_available;
	float dist_correction_x;
	float dist_correction_y;
	float vert_offset_correction;
	float horiz_offset_correction;
	int max_intensity;
	int min_intensity;
	float focal_distance;
	float focal_slope;

	/** cached values calculated when the calibration file is read */
	float cos_rot_correction;              ///< cosine of rot_correction
	float sin_rot_correction;              ///< sine of rot_correction
	float cos_vert_correction;             ///< cosine of vert_correction
	float sin_vert_correction;             ///< sine of vert_correction

	int laser_ring;                        ///< ring number for this laser
};

/** \brief Calibration information for the entire device. */
class Calibration {

public:

	std::map<int, LaserCorrection> laser_corrections;
	int num_lasers;
	bool initialized;
	bool ros_info;

public:

	Calibration(bool info = true) :
			initialized(false), ros_info(info) {
	}
	Calibration(const std::string& calibration_file, bool info = true) :
			ros_info(info) {
		read(calibration_file);
	}

	void read(const std::string& calibration_file);
	void write(const std::string& calibration_file);
};

} /* vanjee_lidar */

#endif /* end of include guard: __VANJEE_CALIBRATION_H */

