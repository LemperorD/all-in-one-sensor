#pragma once
#include <stdint.h>
#include "common/mavlink.h"
// #include "serial_port.hpp"
#include "serial_manager.hpp"

#include <functional>
#include <mutex>
#include <queue>
class MavlinkHandler {
public:
    using SystemtimeCallback = std::function<void(uint64_t time_unix_usec)>;
    using TimesyncCallback = std::function<void(int64_t ts1, int64_t tc1)>;
    using HeartbeatCallback = std::function<void(bool timesync_ok)>;
    using CameratriggerCallback = std::function<void(uint64_t recv_host_usec ,uint64_t time_usec)>;
    using ImuCallback = std::function<void(uint64_t timestamp_us,
                                            float ax, float ay, float az,
                                            float gx, float gy, float gz)>;
    using MagCallback = std::function<void(uint64_t timestamp_us,
                                            float mx, float my, float mz)>;
    using LogCallback =  std::function<void(uint8_t severity, const std::string &text)>;                            

    MavlinkHandler(SerialManager &serial);
    void processIncoming(); // 解析MAVLink消息

    void setSystemtimeCallback(SystemtimeCallback cb) { systemtime_cb_ = cb; }
    void setTimesyncCallback(TimesyncCallback cb) { timesync_cb_ = cb; }
    void setHeartbeatCallback(HeartbeatCallback cb) { heartbeat_cb_ = cb; }
    void setCameratriggerCallback(CameratriggerCallback cb) { cam_trigger_cb_ = cb; }
    void setImuCallback(ImuCallback cb) { imu_cb_ = cb; }
    void setMagCallback(MagCallback cb) { mag_cb_ = cb; }
    void setLogCallback(LogCallback cb) { log_cb_ = cb; }
private:
    std::vector<float> imu_data_buffer_;
    SerialManager &serial_;
    SystemtimeCallback systemtime_cb_;
    TimesyncCallback timesync_cb_;
    HeartbeatCallback heartbeat_cb_;
    CameratriggerCallback cam_trigger_cb_; 
    ImuCallback imu_cb_;
    MagCallback mag_cb_;
    LogCallback log_cb_;
};
