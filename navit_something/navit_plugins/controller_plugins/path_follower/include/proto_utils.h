#include <google/protobuf/message.h>

namespace proto {

// Load a file that contains a serialized proto.
bool loadProtoBinaryFile(const std::string& filename, ::google::protobuf::Message* proto);

// Load a file that contains text format of a proto.
bool loadProtoTextFile(const std::string& filename, ::google::protobuf::Message* proto);

// Load a file with the hint from the extension of filename.
bool loadProtoFile(const std::string& filename, ::google::protobuf::Message* proto);

// Write a serialized proto into file.
bool writeProtoBinaryFile(const std::string& filename, const ::google::protobuf::Message& proto);

// Write text format of a proto into file.
bool writeProtoTextFile(const std::string& filename, const ::google::protobuf::Message& proto);

// Write a file with the hint from the extension of filename.
bool writeProtoFile(const std::string& filename, const ::google::protobuf::Message& proto);

std::string base64_decode(std::string const& encoded_string);

std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);

}  // namespace proto