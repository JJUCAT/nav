#ifndef NOVATEL_GPS_DRIVER_NMEA_SENTENCE_H
#define NOVATEL_GPS_DRIVER_NMEA_SENTENCE_H

#include <string>
#include <vector>

namespace integrated_navigation
{
    /**
    * Contains an NMEA sentence that has been tokenized into a vector of strings.
    */
    struct NmeaSentence
    {
        std::string id;
        std::vector<std::string> body;
    };
}
#endif //NOVATEL_GPS_DRIVER_NMEA_SENTENCE_H