#ifndef NOVATEL_GPS_DRIVER_NOVATEL_SENTENCE_H
#define NOVATEL_GPS_DRIVER_NOVATEL_SENTENCE_H

#include <string>
#include <vector>

namespace integrated_navigation
{
    /**
    * Contains an ASCII NovAtel sentence that has been tokenized into a vector
    * of strings.
    */
    struct NovatelSentence
    {
        std::string id;
        std::vector<std::string> header;
        std::vector<std::string> body;
    };
}
#endif //NOVATEL_GPS_DRIVER_NOVATEL_SENTENCE_H