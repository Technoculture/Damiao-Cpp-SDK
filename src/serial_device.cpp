#include "dm_motor_sdk/serial_device.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include <cstring> // For memset

namespace dm_motor_sdk {

SerialDevice::SerialDevice(const char* port_name, int baud_rate) {
    port_name_ = port_name;
    baud_rate_ = baud_rate;
    is_open_ = false;
#ifdef _WIN32
    handle_ = INVALID_HANDLE_VALUE;
#else
    file_descriptor_ = -1;
#endif
}

SerialDevice::~SerialDevice() {
    if (is_open_) {
        close_device();
    }
}

bool SerialDevice::is_open() const {
    return is_open_;
}

#ifdef _WIN32
// Windows Implementation
SerialErrorCode SerialDevice::open_device() {
    handle_ = CreateFileA(port_name_, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (handle_ == INVALID_HANDLE_VALUE) {
        return SerialErrorCode::OPEN_FAILED;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(handle_, &dcbSerialParams)) {
        CloseHandle(handle_);
        return SerialErrorCode::CONFIG_FAILED;
    }

    dcbSerialParams.BaudRate = baud_rate_;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(handle_, &dcbSerialParams)) {
        CloseHandle(handle_);
        return SerialErrorCode::CONFIG_FAILED;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(handle_, &timeouts)) {
        CloseHandle(handle_);
        return SerialErrorCode::CONFIG_FAILED;
    }

    is_open_ = true;
    return SerialErrorCode::SUCCESS;
}

void SerialDevice::close_device() {
    if (is_open_ && handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(handle_);
        handle_ = INVALID_HANDLE_VALUE;
        is_open_ = false;
    }
}

int SerialDevice::read_data(uint8_t* buffer, int buffer_size) {
    if (!is_open_ || handle_ == INVALID_HANDLE_VALUE) {
        return -1;
    }
    DWORD bytes_read = 0;
    if (!ReadFile(handle_, buffer, buffer_size, &bytes_read, NULL)) {
        return -1;
    }
    return bytes_read;
}

int SerialDevice::write_data(const uint8_t* data, int length) {
    if (!is_open_ || handle_ == INVALID_HANDLE_VALUE) {
        return -1;
    }
    DWORD bytes_written = 0;
    if (!WriteFile(handle_, data, length, &bytes_written, NULL)) {
        return -1;
    }
    return bytes_written;
}

#else
// Linux/macOS Implementation
SerialErrorCode SerialDevice::open_device() {
    file_descriptor_ = open(port_name_, O_RDWR | O_NOCTTY | O_NDELAY);
    if (file_descriptor_ == -1) {
        return SerialErrorCode::OPEN_FAILED;
    }

    struct termios options;
    tcgetattr(file_descriptor_, &options);

    cfsetispeed(&options, baud_rate_);
    cfsetospeed(&options, baud_rate_);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; // 1 second timeout

    if (tcsetattr(file_descriptor_, TCSANOW, &options) != 0) {
        close(file_descriptor_);
        return SerialErrorCode::CONFIG_FAILED;
    }

    is_open_ = true;
    return SerialErrorCode::SUCCESS;
}

void SerialDevice::close_device() {
    if (is_open_ && file_descriptor_ != -1) {
        close(file_descriptor_);
        file_descriptor_ = -1;
        is_open_ = false;
    }
}

int SerialDevice::read_data(uint8_t* buffer, int buffer_size) {
    if (!is_open_ || file_descriptor_ == -1) {
        return -1;
    }
    return read(file_descriptor_, buffer, buffer_size);
}

int SerialDevice::write_data(const uint8_t* data, int length) {
    if (!is_open_ || file_descriptor_ == -1) {
        return -1;
    }
    return write(file_descriptor_, data, length);
}

#endif

} // namespace dm_motor_sdk