#include "serial_port.hpp"
#include <iostream>
#include <cstring>

SerialPort::SerialPort() {}
SerialPort::~SerialPort() { closePort(); }

#ifdef _WIN32
// ======== Windows 平台实现 ========
#include <windows.h>

bool SerialPort::openPort(const std::string &port, int baudrate) {
    closePort(); // 确保之前的端口关闭

    std::string fullPortName;
    if (port.rfind("\\\\.\\", 0) == 0)
        fullPortName = port;
    else
        fullPortName = "\\\\.\\" + port;

    handle_ = CreateFileA(
        fullPortName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,    // no sharing
        NULL, // no security
        OPEN_EXISTING,
        0,
        NULL);

    if (handle_ == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to open port " << port << std::endl;
        handle_ = nullptr;
        return false;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(handle_, &dcbSerialParams)) {
        std::cerr << "GetCommState failed\n";
        closePort();
        return false;
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if (!SetCommState(handle_, &dcbSerialParams)) {
        std::cerr << "SetCommState failed\n";
        closePort();
        return false;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(handle_, &timeouts);

    opened_ = true;
    return true;
}

void SerialPort::closePort() {
    if (handle_) {
        CloseHandle(handle_);
        handle_ = nullptr;
    }
    opened_ = false;
}

bool SerialPort::writeData(const uint8_t *data, size_t len) {
    if (!opened_) return false;
    DWORD bytesWritten;
    if (!WriteFile(handle_, data, static_cast<DWORD>(len), &bytesWritten, NULL))
        return false;
    return bytesWritten == len;
}

int SerialPort::readByte(uint8_t &byte) {
    if (!opened_) return -1;
    DWORD bytesRead = 0;
    if (!ReadFile(handle_, &byte, 1, &bytesRead, NULL))
        return -1;
    return bytesRead == 1 ? 1 : 0;
}

bool SerialPort::configurePort(int baudrate) {
    // 已在 openPort 内配置
    return opened_;
}

#else
// ======== Linux / macOS 平台实现 ========
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

bool SerialPort::openPort(const std::string &port, int baudrate) {
    closePort();

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "Failed to open port " << port << ": " << strerror(errno) << std::endl;
        return false;
    }

    if (!configurePort(baudrate)) {
        closePort();
        return false;
    }

    opened_ = true;
    return true;
}

void SerialPort::closePort() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    opened_ = false;
}

bool SerialPort::writeData(const uint8_t *data, size_t len) {
    if (!opened_) return false;
    ssize_t n = ::write(fd_, data, len);
    return n == static_cast<ssize_t>(len);
}

int SerialPort::readByte(uint8_t &byte) {
    if (!opened_) return -1;
    ssize_t n = ::read(fd_, &byte, 1);
    if (n < 0) return -1;
    return n == 1 ? 1 : 0;
}

bool SerialPort::configurePort(int baudrate) {
    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
        return false;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    // 设置 8N1，关闭流控
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    // 关闭所有输入转换
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR | BRKINT);

    // 关闭所有输出转换
    tty.c_oflag &= ~(OPOST);

    // 关闭本地模式（canonical、回显、信号）
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 非阻塞读取：VMIN=0，VTIME=1（0.1s超时）
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

#endif
