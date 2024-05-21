#ifndef FILEFORMAT_H
#define FILEFORMAT_H

/**
 * \file FileFormat.h
 *
 * \brief This file defines various file formats.
 */

namespace dock_visual_perception
{
/**
 * File format enumeration used when reading / writing configuration
 * files.
 */
typedef enum
{
  /**
   * \brief Default file format.
   *
   * Format is either OPENCV, TEXT or XML depending on load/store function used.
   */
  FILE_FORMAT_DEFAULT,

  /**
   * \brief File format written with cvWrite.
   *
   * File contents depend on the specific load/store function used.
   */
  FILE_FORMAT_OPENCV,

  /**
   * \brief Plain ASCII text file format.
   *
   * File contents depend on the specific load/store function used.
   */
  FILE_FORMAT_TEXT,

  /**
   * \brief XML file format.
   *
   * XML schema depends on the specific load/store function used.
   */
  FILE_FORMAT_XML

} FILE_FORMAT;
}

#endif  // FILEFORMAT_H
