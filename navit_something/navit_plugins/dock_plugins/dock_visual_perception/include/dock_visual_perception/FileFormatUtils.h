#ifndef FILEFORMATUTILS_H
#define FILEFORMATUTILS_H

/**
 * \file FileFormatUtils.h
 *
 * \brief This file implements utilities that assist in reading and writing
 * files.
 */

#include "Ar.h"
#include <opencv2/core.hpp>
#include "tinyxml.h"

namespace dock_visual_perception
{
/** \brief Utility functions for file reading / writing.
 */
class AR_EXPORT FileFormatUtils
{
private:
  /**
   * \brief Reads matrix type, rows and cols from XML element.
   * \return true if XML element appears to be valid; otherwise false.
   */
  static bool decodeXMLMatrix(const TiXmlElement* xml_matrix, int& type,
                              int& rows, int& cols);

public:
  /** \brief Allocates cv::Mat of a correct type and size.
   * \param xml_matrix ar:matrix element.
   * \return cv::Mat that has the correct size for \e parseXMLMatrix.
   */
  static cv::Mat* allocateXMLMatrix(const TiXmlElement* xml_matrix);

  /** \brief Reads contents of ar:matrix into cv::Mat.
   *
   * Parsing fails if the matrix is not the same type or does not have
   * the same number of rows and columns as the XML element.
   *
   * \param xml_matrix ar:matrix element. If NULL no parsing is done and
   *                   false is returned.
   * \param matrix cv::Mat that has the correct size, populated with data in
   *               the xml_matrix.
   * \return true if matrix was successfully parsed; otherwise false.
   */
  static bool parseXMLMatrix(const TiXmlElement* xml_matrix, cv::Mat& matrix);

  /** \brief Allocates new XML element and populates it with a cv::Mat data.
   *
   * The returned element needs to be deallocated by the caller.
   *
   * \param element_name Name of the allocated tiXmlElement.
   * \param matrix Data that is written into the returned XML element.
   * \return Newly allocated TiXmlElement.
   */
  static TiXmlElement* createXMLMatrix(const char* element_name,
                                       const cv::Mat& matrix);
};
}

#endif  // FILEFORMATUTILS_H
