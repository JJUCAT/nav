#include <limits>
#include <sstream>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <ros/ros.h>

#include <integrated_navigation/library/Serial/string_utils.h>
#include <integrated_navigation/library/GnssParserLib/parsers/header.h>
#include <integrated_navigation/library/GnssParserLib/parsers/gpgga.h>
#include <integrated_navigation/library/GnssParserLib/parsers/gprmc.h>
#include <integrated_navigation/library/GnssParserLib/novatel_message_extractor.h>

namespace integrated_navigation
{
  const std::string NovatelMessageExtractor::CHECKSUM_FLAG = "*";
  const std::string NovatelMessageExtractor::FIELD_SEPARATOR = ",";
  const std::string NovatelMessageExtractor::HEADER_SEPARATOR = ";";
  const std::string NovatelMessageExtractor::NMEA_SENTENCE_FLAG = "$";
  const std::string NovatelMessageExtractor::NOVATEL_SENTENCE_FLAG = "#";
  const std::string NovatelMessageExtractor::NOVATEL_ASCII_FLAGS = "$#";
  const std::string NovatelMessageExtractor::NOVATEL_BINARY_SYNC_BYTES = "\xAA\x44\x12";
  const std::string NovatelMessageExtractor::NOVATEL_ENDLINE = "\r\n";
  
  uint32_t NovatelMessageExtractor::CRC32Value(int32_t i)
  {
    int32_t j;
    uint32_t ulCRC;
    ulCRC = static_cast<uint32_t>(i);
    for ( j = 8 ; j > 0; j-- )
    {
      if ( ulCRC & 1u )
        ulCRC = static_cast<uint32_t>(( ulCRC >> 1u ) ^ NOVATEL_CRC32_POLYNOMIAL);
      else
        ulCRC >>= 1u;
    }
    return ulCRC;
  }
const unsigned long crcTable[256] = {
	0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
	0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,
	0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL, 0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
	0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,
	0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
	0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
	0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
	0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L, 0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
	0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
	0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
	0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
	0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
	0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L, 0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
	0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,
	0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
	0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,
	0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
	0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L, 0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
	0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
	0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
	0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
	0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,
	0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L, 0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
	0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
	0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
	0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,
	0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
	0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL, 0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
	0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
	0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
	0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
	0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL
};
  uint32_t NovatelMessageExtractor::CalculateBlockCRC32(
      uint32_t ulCount,          // Number of bytes in the data block
      const uint8_t* ucBuffer )  // Data block
  {
    uint32_t ulTemp1;
    uint32_t ulTemp2;
    uint32_t ulCRC = 0;
    int   iIndex, iSize=ulCount;
    for (iIndex = 0; iIndex < iSize; iIndex++) {
		ulCRC = crcTable[(ulCRC ^ ucBuffer[iIndex]) & 0xff] ^ (ulCRC >> 8);
	}
    // while ( ulCount-- != 0 )
    // {
    //   //ulTemp1 = static_cast<uint32_t>(( ulCRC >> 8u ) & 0x00FFFFFFL);
    //   ulTemp1 = static_cast<uint32_t>(( ulCRC >> 8u ) & 0x0FFFFFFFFL);
    //   ulTemp2 = CRC32Value( ((int32_t) ulCRC ^ *ucBuffer++ ) & 0xffu );
    //   ulCRC = ulTemp1 ^ ulTemp2;
    // }
    return( ulCRC );
  }

  uint8_t NovatelMessageExtractor::NmeaChecksum(const std::string& sentence)
  {
    uint8_t checksum = 0;
    std::string::const_iterator it = sentence.begin();
    for (; it != sentence.end(); ++it)
    {
      checksum ^= *it;
    }
    return checksum;
  }

  size_t NovatelMessageExtractor::GetSentenceChecksumStart(const std::string& str, size_t start_idx)
  {
    return str.find(CHECKSUM_FLAG, start_idx);
  }

  void NovatelMessageExtractor::VectorizeString(
      const std::string& str,
      std::vector<std::string>& vectorized_message,
      const std::string& delimiters)
  {
    boost::algorithm::split(vectorized_message, str, boost::algorithm::is_any_of(delimiters));
  }

  bool NovatelMessageExtractor::GetNovatelMessageParts(
      const std::string& sentence,
      std::string& message_id,
      std::vector<std::string>& header,
      std::vector<std::string>& body)
  {
    message_id.clear();
    header.clear();
    body.clear();

    std::vector<std::string> vectorized_message;
    
    VectorizeString(sentence, vectorized_message, HEADER_SEPARATOR);

    if (vectorized_message.size() != 2)
    {
      return false;
    }

    VectorizeString(vectorized_message[0], header, FIELD_SEPARATOR);
    VectorizeString(vectorized_message[1], body, FIELD_SEPARATOR);

    // VectorizeString(vectorized_message[0], body, FIELD_SEPARATOR);
    // VectorizeString(vectorized_message[1], header, FIELD_SEPARATOR);

    if (!header.empty())
    {
      message_id = header.front();
    }
    else
    {
      return false;
    }
    //body.insert(body.end(),header.begin(),header.end());
    return true;
  }

  int32_t NovatelMessageExtractor::GetBinaryMessage(const std::string& str,
                           size_t start_idx,
                           BinaryMessage& msg)
  {
    if (str.length() < HeaderParser::BINARY_HEADER_LENGTH + 4)
    {
      // The shortest a binary message can be (header + no data + CRC)
      // is 32 bytes, so just return if we don't have at least that many.
      ROS_WARN("Binary message was too short to parse.");
      return -1;
    }

    ROS_DEBUG("Reading binary header.");
    msg.header_.ParseHeader(reinterpret_cast<const uint8_t*>(&str[start_idx]));
    auto data_start = static_cast<uint16_t>(msg.header_.header_length_ + start_idx);
    uint16_t data_length = msg.header_.message_length_;

    if (msg.header_.sync0_ != static_cast<uint8_t>(NOVATEL_BINARY_SYNC_BYTES[0]) ||
        msg.header_.sync1_ != static_cast<uint8_t>(NOVATEL_BINARY_SYNC_BYTES[1]) ||
        msg.header_.sync2_ != static_cast<uint8_t>(NOVATEL_BINARY_SYNC_BYTES[2]))
    {
      ROS_ERROR("Sync bytes were incorrect; this should never happen and is definitely a bug: %x %x %x",
               msg.header_.sync0_, msg.header_.sync1_, msg.header_.sync2_);
      return -2;
    }

    if (msg.header_.header_length_ != HeaderParser::BINARY_HEADER_LENGTH)
    {
      ROS_WARN("Binary header length was unexpected: %u (expected %u)",
               msg.header_.header_length_, HeaderParser::BINARY_HEADER_LENGTH);
    }

    ROS_DEBUG("Msg ID: %u    Data start / length: %u / %u",
              msg.header_.message_id_, data_start, data_length);

    if (data_start + data_length + 4 > (int)str.length())
    {
      ROS_DEBUG("Not enough data.");
      return -1;
    }

    ROS_DEBUG("Reading binary message data.");
    msg.data_.resize(data_length);
    std::copy(&str[data_start], &str[data_start+data_length], reinterpret_cast<char*>(&msg.data_[0]));

    ROS_DEBUG("Calculating CRC.");

    uint32_t crc = CalculateBlockCRC32(static_cast<uint32_t>(msg.header_.header_length_) + data_length,
                                       reinterpret_cast<const uint8_t*>(&str[start_idx]));

    ROS_DEBUG("Reading CRC.");
    msg.crc_ = ParseUInt32(reinterpret_cast<const uint8_t*>(&str[data_start+data_length]));

    if (crc != msg.crc_)
    {
      // Invalid CRC
      ROS_DEBUG("Invalid CRC;  Calc: %u    In msg: %u", crc, msg.crc_);
      return -2;
    }

    // On success, return how many bytes we read

    ROS_DEBUG("Finishing reading binary message.");
    return static_cast<int32_t>(msg.header_.header_length_ + data_length + 4);
  }

  int32_t NovatelMessageExtractor::GetNovatelSentence(
      const std::string& str,
      size_t start_idx,
      std::string& sentence)
  {
    sentence.clear();

    size_t checksum_start = GetSentenceChecksumStart(str, start_idx);
    if (checksum_start == std::string::npos)
    {
      // Sentence not complete. Just return.
      return -1;
    }
    else if (checksum_start + 8 >= str.size())
    {
      // Sentence not complete. Just return.
      return -1;
    }
    else
    {
      // Compare the checksums
      sentence = str.substr(start_idx + 1, checksum_start - start_idx - 1);
      std::string checksum_str = str.substr(checksum_start + 1, 8);
      uint64_t checksum = std::strtoul(checksum_str.c_str(), nullptr, 16);
      uint64_t calculated_checksum = CalculateBlockCRC32(
          static_cast<uint32_t>(sentence.size()),
          reinterpret_cast<const uint8_t*>(sentence.c_str()));
      

      if (checksum == ULONG_MAX)
      {
        // Invalid checksum -- strtoul failed
        return 1;
      }
      else if(static_cast<uint32_t>(checksum) == calculated_checksum)
      {
        return 0;
      }
      else
      {
        ROS_WARN("Expected checksum: [%lx]", calculated_checksum);
        // Invalid checksum
        return 1;
      }
    }
  }

  int32_t NovatelMessageExtractor::GetNmeaSentence(
      const std::string& str,
      size_t start_idx,
      std::string& sentence,
      bool keep_container)
  {
    sentence.clear();

    size_t checksum_start = GetSentenceChecksumStart(str, start_idx);
   
    if (checksum_start == std::string::npos)
    {
      // Sentence not complete. Just return.
      return -1;
    }
    else if (checksum_start + 2 >= str.size())
    {
      // Sentence not complete. Just return.
      return -1;
    }
    else
    {
      // Compare the checksums
      sentence = str.substr(start_idx + 1, checksum_start - start_idx - 1);
      std::string checksum_str = str.substr(checksum_start + 1, 2);
      uint64_t checksum = std::strtoul(checksum_str.c_str(), nullptr, 16);
      uint64_t calculated_checksum = NmeaChecksum(sentence);
      if (checksum == ULONG_MAX)
      {
        // Invalid checksum
        return 1;
      }
      else if(static_cast<uint32_t>(checksum) == calculated_checksum)
      {
        if (keep_container)
        {
          sentence.insert(0, "$");
          std::string recreated_checksum_str("*");
          recreated_checksum_str += checksum_str;
          sentence.insert(sentence.end(),
              recreated_checksum_str.begin(),
              recreated_checksum_str.end());
        }
        return 0;
      }
      else
      {
        ROS_WARN("Expected: [%lx]", calculated_checksum);
        // Invalid checksum
        return 1;
      }
    }
  }

  void NovatelMessageExtractor::FindAsciiSentence(const std::string& sentence,
                         size_t current_idx,
                         size_t& start_idx,
                         size_t& end_idx,
                         size_t& invalid_char_idx)
  {
    start_idx = sentence.find_first_of(NOVATEL_ASCII_FLAGS, current_idx);
   
    end_idx = std::string::npos;
    invalid_char_idx = std::string::npos;
// size_t star_idx= sentence.find_first_of("*", current_idx);
// std::cout<<"sentence="<<sentence<<" start_idx="<<start_idx<<" *_idx="<<star_idx<<" sentence_len="<<sentence.length()<<std::endl;
    if (start_idx == std::string::npos)
    {
      return;
    }
    
    end_idx = sentence.find(NOVATEL_ENDLINE, start_idx);
    //std::cout<<"end_idx="<<end_idx<< " "<<sentence[end_idx-1]<<" sentence_len="<<sentence.length()<<std::endl;

    size_t search_stop_idx = std::min(end_idx, sentence.length());
    //std::cout<<"start_idx="<<start_idx<<" end_idx="<<end_idx<<" search_stop_idx="<<search_stop_idx<<std::endl;
    for (size_t i = start_idx; i < search_stop_idx; i++)
    {
      if (sentence[i] == 9 || sentence[i] == 10 || sentence[i] == 11 || sentence[i] == 13 ||
          (sentence[i] >= 32 && sentence[i] <= 126))
      {
        continue;
      }
     
      invalid_char_idx = i;
      std::cout<<"find invalid_char_idx "<<i<<":"<<sentence[invalid_char_idx]<<std::endl;
      // for(int ii=start_idx;ii<100;ii++)
      //  std::cout<<"ii="<<ii<<":"<<sentence[ii]<<std::endl;
      break;
    }
  }

  bool NovatelMessageExtractor::VectorizeNovatelSentence(
      const std::string& data,
      NovatelSentence& sentence)
  {
    return GetNovatelMessageParts(
        data, sentence.id, sentence.header, sentence.body);
  }

  void NovatelMessageExtractor::VectorizeNmeaSentence(
    const std::string& sentence,
    NmeaSentence& vectorized_message)
  {
    VectorizeString(sentence, vectorized_message.body, FIELD_SEPARATOR);
    if (!vectorized_message.body.empty())
    {
      vectorized_message.id = vectorized_message.body.front();
    }
  }

  bool NovatelMessageExtractor::ExtractCompleteMessages(
      const std::string& input,
      std::vector<NmeaSentence>& nmea_sentences,
      std::vector<NovatelSentence>& novatel_sentences,
      std::vector<BinaryMessage>& binary_messages,
      std::string& remaining,
      bool keep_nmea_container)
  {
    bool parse_error = false;

    size_t sentence_start = 0;

    while(sentence_start != std::string::npos && sentence_start < input.size())
    {
      size_t ascii_start_idx;
      size_t ascii_end_idx;
      size_t invalid_ascii_idx;
      sentence_start=0;
      
      FindAsciiSentence(input, sentence_start, ascii_start_idx, ascii_end_idx, invalid_ascii_idx);

      {
        // If we saw the beginning of an ASCII message, try to parse it.
        size_t ascii_len = ascii_end_idx - ascii_start_idx;
        if (invalid_ascii_idx != std::string::npos)
        {
          // If we see an invalid character, don't even bother trying to parse
          // the rest of the message.  By this point we also know there's no
          // binary header before this point, so just skip the data.
          ROS_WARN("Invalid ASCII char: [%s]", input.substr(ascii_start_idx, ascii_len).c_str());
          ROS_WARN("                     %s^", std::string(invalid_ascii_idx-ascii_start_idx-1, ' ').c_str());
          // printf("Binary start: %lu   ASCII start / end / invalid: %lu / %lu / %lu",
          //        binary_start_idx, ascii_start_idx, ascii_end_idx, invalid_ascii_idx);
          sentence_start += invalid_ascii_idx + 1;
          return false;
        }
        else if (ascii_end_idx != std::string::npos)
        {
          // If we've got a start, an end, and no invalid characters, we've
          // got a valid ASCII message.
          ROS_DEBUG("ASCII sentence:\n[%s]", input.substr(ascii_start_idx, ascii_len).c_str());
          if (input[ascii_start_idx] == NMEA_SENTENCE_FLAG[0])
          {  //std::cout<<"enter GetNmeaSentence"<<std::endl;
            std::string cur_sentence;
            int32_t result = GetNmeaSentence(
                input,
                ascii_start_idx,
                cur_sentence,
                keep_nmea_container);
            if (result == 0)
            {
              nmea_sentences.emplace_back(NmeaSentence());
              VectorizeNmeaSentence(cur_sentence, nmea_sentences.back());
              sentence_start = ascii_end_idx;
              return true;
            }
            else if (result < 0)
            {
              // Sentence is not complete, add it to the remaining and break.
              // This is legacy code from before FindAsciiSentence was implemented,
              // and it will probably never happen, but it doesn't hurt anything to
              // have it here.
              remaining = input.substr(ascii_start_idx);
              ROS_DEBUG("Waiting for more NMEA data.");
               return false;
              break;
            }
            else
            {
              ROS_WARN("Invalid NMEA checksum for: [%s]",
                       input.substr(ascii_start_idx, ascii_len).c_str());
              // Sentence had an invalid checksum, just iterate to the next sentence
              sentence_start += 1;
              parse_error = true;
              return false;
            }
          }
          else if (input[ascii_start_idx] == NOVATEL_SENTENCE_FLAG[0])
          {
            std::string cur_sentence;
            int32_t result = GetNovatelSentence(input, ascii_start_idx, cur_sentence);
            if (result == 0)
            {
              // Send to parser for testing:
              novatel_sentences.emplace_back(NovatelSentence());
              if (!VectorizeNovatelSentence(cur_sentence, novatel_sentences.back()))
              {
                novatel_sentences.pop_back();
                parse_error = true;
                ROS_ERROR_THROTTLE(1.0, "Unable to vectorize novatel sentence");
              }
              sentence_start = ascii_end_idx;
             //std::cout<<"VectorizeNovatelSentence success, novatel_sentences.size="<<novatel_sentences.size()<<std::endl;
               return true;
            }
            else if (result < 0)
            {
              // Sentence is not complete, add it to the remaining and break.
              // This is legacy code from before FindAsciiSentence was implemented,
              // and it will probably never happen, but it doesn't hurt anything to
              // have it here.
              remaining = input.substr(ascii_start_idx);
              ROS_INFO("Waiting for more NovAtel data.");
              return false;
              break;
            }
            else
            {
              ROS_WARN("Invalid NovAtel checksum for: [%s]",
                       input.substr(ascii_start_idx, ascii_len).c_str());
              parse_error = true;
              return false;
            }
          }
        }
        else
        {
          ROS_DEBUG("Incomplete ASCII sentence, waiting for more.");
          remaining = input.substr(ascii_start_idx);
           return false;
          break;
        }
      }
    }

    return !parse_error;
  }

  double NovatelMessageExtractor::GetMostRecentUtcTime(const std::vector<NmeaSentence>& sentences)
  {
    std::vector<NmeaSentence>::const_reverse_iterator iter;
    for (iter = sentences.rbegin(); iter != sentences.rend(); iter++)
    {
      if (iter->id == GpggaParser::MESSAGE_NAME || iter->id == GprmcParser::MESSAGE_NAME)
      {
        if (iter->body.size() > 1)
        {
          if (iter->body[1].empty() || iter->body[1] == "0")
          {
            return 0;
          }
          else
          {
            double utc_float;
            if (ToDouble(iter->body[1], utc_float))
            {
              return UtcFloatToSeconds(utc_float);
            }
            return 0;
          }
        }
      }
    }

    return 0;
  }
}
