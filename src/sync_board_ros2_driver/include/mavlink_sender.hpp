#pragma once
// #include "serial_port.hpp"
#include "serial_manager.hpp"

#include "common/mavlink.h"

class MavlinkSender {
public:
    MavlinkSender(SerialManager &serial);

    void sendCameraTrigger(int continuesly, float trigger_interval_ms, int trigger_mode, int trigger_edge);
    void sendTimeSource(int time_source);
    void upgradeProcess(const std::string& upgrade_file_path);

private:
    SerialManager &serial_;
};