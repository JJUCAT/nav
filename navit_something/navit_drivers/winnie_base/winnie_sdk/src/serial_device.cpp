#include <winnie_sdk/hardware/serial_device.h>

#include <utility>

namespace winnie_sdk {
SerialDevice::SerialDevice(std::string port_name, int baud_rate)
    : port_name_(std::move(port_name)), baudrate_(baud_rate), data_bits_(8), parity_bits_('N'), stop_bits_(1) {}

SerialDevice::~SerialDevice() { CloseDevice(); }

bool SerialDevice::Init() {
    std::cout << "Attempting to open device " << port_name_ << " with baudrate " << baudrate_ << std::endl;
    if (port_name_.c_str() == nullptr) {
        port_name_ = "/dev/ttyUSB0";
    }
    if (OpenDevice() && ConfigDevice()) {
        FD_ZERO(&serial_fd_set_);
        FD_SET(serial_fd_, &serial_fd_set_);
        std::cout << "...Serial started successfully." << std::endl;
        return true;
    } else {
        std::cout << "...Failed to start serial " << port_name_ << std::endl;
        CloseDevice();
        return false;
    }
}

bool SerialDevice::OpenDevice() {
#ifdef __arm__
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NONBLOCK);
#elif __x86_64__
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#else
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#endif

    if (serial_fd_ < 0) {
        std::cout << "cannot open device " << serial_fd_ << " " << port_name_ << std::endl;
        return false;
    }

    return true;
}

bool SerialDevice::CloseDevice() {
    close(serial_fd_);
    serial_fd_ = -1;
    return true;
}

bool SerialDevice::ConfigDevice() {
    int st_baud[]  = {B4800, B9600, B19200, B38400, B57600, B115200, B230400, B921600, B1000000, B1152000, B3000000};
    int std_rate[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 921600, 1000000, 1152000, 3000000};
    int i, j;
    /* save current port parameter */
    if (tcgetattr(serial_fd_, &old_termios_) != 0) {
        std::cout << "fail to save current port" << std::endl;
        return false;
    }
    memset(&new_termios_, 0, sizeof(new_termios_));

    /* config the size of char */
    new_termios_.c_cflag |= CLOCAL | CREAD;
    new_termios_.c_cflag &= ~CSIZE;

    /* config data bit */
    switch (data_bits_) {
        case 7:
            new_termios_.c_cflag |= CS7;
            break;
        case 8:
            new_termios_.c_cflag |= CS8;
            break;
        default:
            new_termios_.c_cflag |= CS8;
            break;  // 8N1 default config
    }
    /* config the parity bit */
    switch (parity_bits_) {
        /* odd */
        case 'O':
        case 'o':
            new_termios_.c_cflag |= PARENB;
            new_termios_.c_cflag |= PARODD;
            break;
            /* even */
        case 'E':
        case 'e':
            new_termios_.c_cflag |= PARENB;
            new_termios_.c_cflag &= ~PARODD;
            break;
            /* none */
        case 'N':
        case 'n':
            new_termios_.c_cflag &= ~PARENB;
            break;
        default:
            new_termios_.c_cflag &= ~PARENB;
            break;  // 8N1 default config
    }
    /* config baudrate */
    j = sizeof(std_rate) / 4;
    for (i = 0; i < j; ++i) {
        if (std_rate[i] == baudrate_) {
            /* set standard baudrate */
            cfsetispeed(&new_termios_, st_baud[i]);
            cfsetospeed(&new_termios_, st_baud[i]);
            break;
        }
    }
    /* config stop bit */
    if (stop_bits_ == 1)
        new_termios_.c_cflag &= ~CSTOPB;
    else if (stop_bits_ == 2)
        new_termios_.c_cflag |= CSTOPB;
    else
        new_termios_.c_cflag &= ~CSTOPB;  // 8N1 default config

                                          /* config waiting time & min number of char */
    new_termios_.c_cc[VTIME] = 1;
    new_termios_.c_cc[VMIN]  = 18;

    /* using the raw data mode */
    new_termios_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_termios_.c_oflag &= ~OPOST;

    /* flush the hardware fifo */
    tcflush(serial_fd_, TCIFLUSH);

    /* activite the configuration */
    if ((tcsetattr(serial_fd_, TCSANOW, &new_termios_)) != 0) {
        std::cout << "failed to activate serial configuration" << std::endl;
        return false;
    }
    return true;
}

size_t SerialDevice::Read(uint8_t *buf, size_t len) {
    size_t ret = -1;

    if (NULL == buf) {
        return -1;
    } else {
        ret = read(serial_fd_, buf, len);
        // std::cout<<"Read once length: "<<ret;
        while (ret == 0) {
            std::cout << "Connection closed, try to reconnect." << std::endl;
            while (!Init()) {
                usleep(500000);
            }
            std::cout << "Reconnect Success." << std::endl;
            ret = read(serial_fd_, buf, len);
        }
        return ret;
    }
}

size_t SerialDevice::Write(const uint8_t *buf, size_t len) { return write(serial_fd_, buf, len); }
}  // namespace winnie_sdk
