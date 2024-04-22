#ifndef BT_EXCEPTION_HPP_
#define BT_EXCEPTION_HPP_

#include <string>
#include <memory>

namespace navit_bt_nodes {

class BT_Exception : public std::exception
{
 public:
  BT_Exception(std::string_view message) :
    message_(static_cast<std::string>(message))
  {}

  template <typename... SV>
  BT_Exception(const SV&... args) : message_(str_format(args...))
  {}

  template<typename ... Args>
  std::string str_format(const std::string &format, Args ... args)
  {
    auto size_buf = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1; 
    std::unique_ptr<char[]> buf(new(std::nothrow) char[size_buf]);

    if (!buf)
      return std::string("");

    std::snprintf(buf.get(), size_buf, format.c_str(), args ...);
    return std::string(buf.get(), buf.get() + size_buf - 1); 
  }

  const char* what() const noexcept
  {
    return message_.c_str();
  }

 private:
  std::string message_;

}; // class BT_Exception

}  // namespace navit_bt_nodes

#endif  // BT_EXCEPTION_HPP_
