#ifndef WINNIE_SDK_SERIAL_DEVICE_H
#define WINNIE_SDK_SERIAL_DEVICE_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <winnie_sdk/hardware/hardware_interface.h>
#include <winnie_sdk/utilities/log.h>

#include <cstring>
#include <string>

namespace winnie_sdk {
/**
 * @brief serial device class inherited from hardware interface
 */
class SerialDevice : public HardwareInterface {
   public:
    /**
     * @brief Constructor of serial device
     * @param port_name port name, i.e. /dev/ttyUSB0
     * @param baudrate serial baudrate
     */
    SerialDevice(std::string port_name, int baud_rate);

    /**
     * @brief Destructor of serial device to close the device
     */
    ~SerialDevice() override;

    /**
     * @brief Initialization of serial device to config and open the device
     * @return True if success
     */
    bool Init() override;

    /**
     * @brief Serial device read function
     * @param buf Given buffer to be updated by reading
     * @param len Read data length
     * @return -1 if failed, else the read length
     */
    size_t Read(uint8_t *buf, size_t len) override;

    /**
     * @brief Write the buffer data into device to send the data
     * @param buf Given buffer to be sent
     * @param len Send data length
     * @return < 0 if failed, else the send length
     */
    size_t Write(const uint8_t *buf, size_t len) override;

   private:
    /**
     * @brief Open the serial device
     * @return True if open successfully
     */
    bool OpenDevice();

    /**
     * @brief Close the serial device
     * @return True if close successfully
     */
    bool CloseDevice();

    /**
     * @brief Configure the device
     * @return True if configure successfully
     */
    bool ConfigDevice();

    //! port name of the serial device
    std::string port_name_;
    //! baudrate of the serial device
    int baudrate_;
    //! stop bits of the serial device, as default
    int stop_bits_;
    //! data bits of the serial device, as default
    int data_bits_;
    //! parity bits of the serial device, as default
    char parity_bits_;
    //! serial handler
    int serial_fd_{};
    //! set flag of serial handler
    fd_set serial_fd_set_{};
    //! termios config for serial handler
    struct termios new_termios_{}, old_termios_{};
};
}  // namespace winnie_sdk
#endif  // WINNIE_SDK_SERIAL_DEVICE_H
