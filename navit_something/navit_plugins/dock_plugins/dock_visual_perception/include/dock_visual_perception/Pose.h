#ifndef POSE_H
#define POSE_H

#include "Ar.h"
#include "Rotation.h"

/**
 * \file Pose.h
 *
 * \brief This file implements a pose.
 */

namespace dock_visual_perception
{
/**
 * \brief \e Pose representation derived from the \e Rotation class
 *
 * The rotation part of the transformation is handled by \e Rotation .
 * The translation part is stored internally using homogeneous 4-vector.
 *
 * Internally in AR we assume coordinate system where
 * 'x' is right, 'y' is down, and 'z' is forward. However
 * the \e SetMatrixGL and \e GetMatrixGL change the pose
 * to support coordinates commonly used in OpenGL:
 * 'x' is right, 'y' is up, and 'z' is backward.
 */
class AR_EXPORT Pose : public Rotation
{
protected:
  // Note, although we are using homogeneous coordinates x, y, z, w --  w is now
  // mostly ignored
public:
  double translation[4];
  cv::Mat translation_mat;

  /** \e Output for debugging purposes */
  void Output() const;
  /** \e Constructor */
  Pose();
  /** \e Constructor using the given translation and rotation elements
   *  \param tra Column vector containing three translation elements
   *  \param rot Handled using the \e Rotation class
   *  \param t   Handled using the \e Rotation class
   */
  Pose(const cv::Mat& tra, const cv::Mat& rot, RotationType t);
  /** \e Constructor with 3x3, 3x4 or 4x4 matrix representation
   *  \param mat A 3x3 rotation matrix or 3x4 / 4x4 transformation matrix
   */
  Pose(const cv::Mat& mat);
  /** \e Copy constructor */
  Pose(const Pose& p);
  /** \e Reset the pose */
  void Reset();
  /** Set the transformation from the given matrix \e mat
   *  \param mat A 3x3 rotation matrix or 3x4 / 4x4 transformation matrix
   */
  void SetMatrix(const cv::Mat& mat);
  /** \brief Set the \e Pose using OpenGL's transposed format.
   *  Note, that by default this also mirrors both the y- and z-axis (see \e
   * Camera and \e Pose for more information) \param gl OpenGL 4x4
   * transformation matrix elements in column-order
   */
  void SetMatrixGL(double gl[16], bool mirror = true);
  /** Get the transformation into the given matrix \e mat
   *  \param mat A 3x3 rotation matrix or 3x4 / 4x4 transformation matrix
   */
  void GetMatrix(cv::Mat& mat) const;
  /** \brief Get the transformation matrix representation of the \e Pose using
   * OpenGL's transposed format. Note, that by default this also mirrors both
   * the y- and z-axis (see \e Camera and \e Pose for more information) \param
   * gl OpenGL 4x4 transformation matrix elements in column-order
   */
  void GetMatrixGL(double gl[16], bool mirror = true);
  /** \e Transpose the transformation */
  void Transpose();
  /** Invert the pose */
  void Invert();
  /** \e Mirror the \e Pose
   *  \param x If true mirror the x-coordinates
   *  \param y If true mirror the y-coordinates
   *  \param z If true mirror the z-coordinates
   */
  void Mirror(bool x, bool y, bool z);
  /** Set the translation part for the \e Pose
   *  \param tra Column vector containing three translation elements
   */
  void SetTranslation(const cv::Mat& tra);
  /** Set the translation part for the \e Pose
   *  \param tra Array containing three translation elements
   */
  void SetTranslation(const double* tra);
  /** Set the translation part for the \e Pose */
  void SetTranslation(const double x, const double y, const double z);
  /** Get the translation part from the \e Pose
   *  \param tra Column vector where the three translation elements are filled
   * in
   */
  void GetTranslation(cv::Mat& tra) const;
  /** Assignment operator for copying \e Pose class */
  Pose& operator=(const Pose& p);
};

}

#endif
