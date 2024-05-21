#ifndef AR_EXCEPTION_H
#define AR_EXCEPTION_H

/**
 * \file ArException.h
 *
 * \brief This file implements the AR exception class.
 */

#include <stdexcept>

namespace dock_visual_perception
{
/**
 * \brief AR exception class.
 */
class ArException : public std::runtime_error
{
public:
  ArException(const char* s) : std::runtime_error(s)
  {
  }
};

}

#endif
