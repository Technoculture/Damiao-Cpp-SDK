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

SerialDevice::SerialDevice(const char* new_port_name, int new_baud_rate) {
    this->port_name = new_port_name;
    this->baud_rate = new_baud_rate;
    this->is_device_open = false;
#ifdef _WIN32
    this->handle = INVALID_HANDLE_VALUE;
#else
    this->file_descriptor = -1;
#endif
}

SerialDevice::~SerialDevice() {
    if (is_device_open) {
        close_device();
    }
}

bool SerialDevice::is_open() const {
    return is_device_open;
}

#ifdef _WIN32
// Windows Implementation
ErrorCode SerialDevice::open_device() {
    handle = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (handle == INVALID_HANDLE_VALUE) {
        return ErrorCode::OPEN_FAILED;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(handle, &dcbSerialParams)) {
        CloseHandle(handle);
        return ErrorCode::CONFIG_FAILED;
    }

    dcbSerialParams.BaudRate = baud_rate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(handle, &dcbSerialParams)) {
        CloseHandle(handle);
        return ErrorCode::CONFIG_FAILED;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(handle, &timeouts)) {
        CloseHandle(handle);
        return ErrorCode::CONFIG_FAILED;
    }

    is_device_open = true;
    return ErrorCode::SUCCESS;
}

void SerialDevice::close_device() {
    if (is_device_open && handle != INVALID_HANDLE_VALUE) {
        CloseHandle(handle);
        handle = INVALID_HANDLE_VALUE;
        is_device_open = false;
    }
}

int SerialDevice::read_data(uint8_t* buffer, int buffer_size) {
    if (!is_device_open || handle == INVALID_HANDLE_VALUE) {
        return -1;
    }
    DWORD bytes_read = 0;
    if (!ReadFile(handle, buffer, buffer_size, &bytes_read, NULL)) {
        return -1;
    }
    return bytes_read;
}

int SerialDevice::write_data(const uint8_t* data, int length) {
    if (!is_device_open || handle == INVALID_HANDLE_VALUE) {
        return -1;
    }
    DWORD bytes_written = 0;
    if (!WriteFile(handle, data, length, &bytes_written, NULL)) {
        return -1;
    }
    return bytes_written;
}

#else
// Linux/macOS Implementation
ErrorCode SerialDevice::open_device() {
    file_descriptor = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (file_descriptor == -1) {
        return ErrorCode::OPEN_FAILED;
    }

    struct termios options;
    tcgetattr(file_descriptor, &options);

    cfsetispeed(&options, static_cast<speed_t>(baud_rate));
    cfsetospeed(&options, static_cast<speed_t>(baud_rate));

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~static_cast<tcflag_t>(PARENB);
    options.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
    options.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
    options.c_cflag |= CS8;
    options.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
    options.c_oflag &= ~static_cast<tcflag_t>(OPOST);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; // 1 second timeout

    if (tcsetattr(file_descriptor, TCSANOW, &options) != 0) {
        close(file_descriptor);
        return ErrorCode::CONFIG_FAILED;
    }

    is_device_open = true;
    return ErrorCode::SUCCESS;
}

void SerialDevice::close_device() {
    if (is_device_open && file_descriptor != -1) {
        close(file_descriptor);
        file_descriptor = -1;
        is_device_open = false;
    }
}

int SerialDevice::read_data(uint8_t* buffer, int buffer_size) {
    if (!is_device_open || file_descriptor == -1) {
        return -1;
    }
    ssize_t bytes_read = read(file_descriptor, buffer, static_cast<size_t>(buffer_size));
    return static_cast<int>(bytes_read);
}

int SerialDevice::write_data(const uint8_t* data, int length) {
    if (!is_device_open || file_descriptor == -1) {
        return -1;
    }
    ssize_t bytes_written = write(file_descriptor, data, static_cast<size_t>(length));
    return static_cast<int>(bytes_written);
}

#endif

} // namespace dm_motor_sdk