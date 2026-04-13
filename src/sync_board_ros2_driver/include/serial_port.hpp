#pragma once
#include <string>
#include <stdint.h>

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool openPort(const std::string &port, int baudrate);
    void closePort();

    // 逐字节读取
    int readByte(uint8_t &byte);

    // 批量写入
    bool writeData(const uint8_t *data, size_t len);

    bool isOpen() const { return opened_; }

private:
#ifdef _WIN32
    void* handle_= nullptr;  
#else
    int fd_ = -1;        
#endif

    bool opened_ = false;
    bool configurePort(int baudrate);
};
