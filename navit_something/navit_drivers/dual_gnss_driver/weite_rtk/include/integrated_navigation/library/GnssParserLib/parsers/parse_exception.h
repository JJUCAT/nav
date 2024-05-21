#ifndef NOVATEL_GPS_DRIVER_PARSE_EXCEPTION_H
#define NOVATEL_GPS_DRIVER_PARSE_EXCEPTION_H

#include <exception>

namespace integrated_navigation
{
    /**
    * Thrown whenever a parser class has an unrecoverable issue  parsing a message.
    */
    class ParseException : public std::runtime_error
    {
    public:
        explicit ParseException(const std::string& error) : std::runtime_error(error)
        {}
    };
}
#endif //NOVATEL_GPS_DRIVER_PARSE_EXCEPTION_H