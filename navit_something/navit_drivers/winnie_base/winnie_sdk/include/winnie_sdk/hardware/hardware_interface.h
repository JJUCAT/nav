#ifndef WINNIE_SDK_HARDWARE_INTERFACE_H
#define WINNIE_SDK_HARDWARE_INTERFACE_H

#include <string>

namespace winnie_sdk {
/**
 * @brief Abstract class for hardware as an interface
 */
class HardwareInterface {
   public:
    HardwareInterface()          = default;
    virtual ~HardwareInterface() = default;

    virtual bool Init()                                  = 0;
    virtual size_t Read(uint8_t *buf, size_t len)        = 0;
    virtual size_t Write(const uint8_t *buf, size_t len) = 0;
};
}  // namespace winnie_sdk
#endif  // WINNIE_SDK_HARDWARE_INTERFACE_H
