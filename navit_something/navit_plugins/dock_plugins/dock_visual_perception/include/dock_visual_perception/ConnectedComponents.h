#ifndef CONNECTEDCOMPONENTS_H
#define CONNECTEDCOMPONENTS_H

/**
 * \file ConnectedComponents.h
 *
 * \brief This file implements connected component labeling.
 */

#include "Ar.h"
#include "Util.h"
#include "Line.h"
#include "Camera.h"

namespace dock_visual_perception
{
/**
 * \brief Connected components labeling methods.
 */
enum AR_EXPORT LabelingMethod
{
  CVSEQ
};

/**
 * \brief Base class for labeling connected components from binary image.
 */
class AR_EXPORT Labeling
{
protected:
  Camera* cam;
  int thresh_param1, thresh_param2;

public:
  /**
   * \brief Pointer to grayscale image that is thresholded for labeling.
   */
  cv::Mat gray;
  /**
   * \brief Pointer to binary image that is then labeled.
   */
  cv::Mat bw;

  /**
   * \brief Vector of 4-length vectors where the corners of detected blobs are
   * stored.
   */
  std::vector<std::vector<PointDouble>> blob_corners;

  /**
   * \brief Two alternatives for thresholding the gray image. ADAPT (adaptive
   * threshold) is only supported currently.
   */
  enum ThresholdMethod
  {
    THRESH,
    ADAPT
  };

  /** Constructor */
  Labeling();

  /** Destructor*/
  virtual ~Labeling();

  /**
   * \brief Sets the camera object that is used to correct lens distortions.
   */
  void SetCamera(Camera* camera)
  {
    cam = camera;
  }

  /**
   * \brief Labels image and filters blobs to obtain square-shaped objects from
   * the scene.
   */
  virtual void LabelSquares(cv::Mat& image, bool visualize = false) = 0;

  bool CheckBorder(const std::vector<cv::Point>& contour, int width,
                   int height);

  void SetThreshParams(int param1, int param2)
  {
    thresh_param1 = param1;
    thresh_param2 = param2;
  }
};

/**
 * \brief Labeling class that uses OpenCV routines to find connected components.
 */
class AR_EXPORT LabelingCvSeq : public Labeling
{
protected:
  int _n_blobs;
  int _min_edge;
  int _min_area;
  bool detect_pose_grayscale;

public:
  LabelingCvSeq();
  ~LabelingCvSeq();

  void SetOptions(bool _detect_pose_grayscale = false);

  void LabelSquares(cv::Mat& image, bool visualize = false);

  std::vector<std::vector<cv::Point>> LabelImage(cv::Mat& image, int min_size,
                                                 bool approx = false);
};

}

#endif
