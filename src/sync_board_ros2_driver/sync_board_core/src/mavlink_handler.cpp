#include "mavlink_handler.hpp"
#include <iostream>
#include <time.h>
#include <chrono>
#include <iomanip>  
#define CUSTOM_MODE_TIMESYNC_OK (1 << 0)

MavlinkHandler::MavlinkHandler(SerialManager &serial) : serial_(serial) {}

void MavlinkHandler::processIncoming() {
    uint8_t byte;
    static mavlink_message_t msg;
    static mavlink_status_t status;

    while (serial_.readByte(byte) == 1) {
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_SYSTEM_TIME: {
                    mavlink_system_time_t system_time;
                    mavlink_msg_system_time_decode(&msg, &system_time);
                    if (systemtime_cb_) systemtime_cb_(system_time.time_unix_usec);
                    break;
                }
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    bool timesync_ok = hb.custom_mode & CUSTOM_MODE_TIMESYNC_OK;
                    if (heartbeat_cb_) heartbeat_cb_(timesync_ok);
                    break;
                }
                case MAVLINK_MSG_ID_STATUSTEXT: {
                    mavlink_statustext_t statustext;
                    mavlink_msg_statustext_decode(&msg, &statustext);
                    std::string text(reinterpret_cast<char*>(statustext.text));
                    size_t pos = text.find('\n');
                    if (pos != std::string::npos) {
                        text = text.substr(0, pos);
                    }
                    if (log_cb_) log_cb_(statustext.severity, text);
                    // std::cout << "STATUSTEXT: " << text << std::endl;
                    break;
                }
                case MAVLINK_MSG_ID_TIMESYNC: {
                    mavlink_timesync_t ts;
                    mavlink_msg_timesync_decode(&msg, &ts);
                    auto now = std::chrono::system_clock::now();
                    auto us  = std::chrono::time_point_cast<std::chrono::microseconds>(now);
                    // 回复TIMESYNC
                    mavlink_timesync_t ts_resp;
                    ts_resp.ts1 = ts.ts1;
                    ts_resp.tc1 = us.time_since_epoch().count(); // 用系统时间
                    mavlink_message_t resp_msg;
                    mavlink_msg_timesync_encode(1, 200, &resp_msg, &ts_resp);
                    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                    uint16_t len = mavlink_msg_to_send_buffer(buf, &resp_msg);
                    serial_.writeData(buf, len);
                    if (timesync_cb_) timesync_cb_(ts.ts1, ts_resp.tc1);
                    break;
                }
                case MAVLINK_MSG_ID_CAMERA_TRIGGER: {
                    mavlink_camera_trigger_t camera_trigger;
                    mavlink_msg_camera_trigger_decode(&msg, &camera_trigger);         // us部分
                    if (cam_trigger_cb_) {
                        auto now = std::chrono::system_clock::now();
                        auto us  = std::chrono::time_point_cast<std::chrono::microseconds>(now);
                        uint64_t now_usec = static_cast<uint64_t>(us.time_since_epoch().count());
                        cam_trigger_cb_(now_usec,camera_trigger.time_usec);

                        int64_t trigger_usec = static_cast<int64_t>(camera_trigger.time_usec);
                        
                    }
                    break;
                }
                case MAVLINK_MSG_ID_HIGHRES_IMU: {
                    mavlink_highres_imu_t highres_imu;
                    mavlink_msg_highres_imu_decode(&msg, &highres_imu);
                    if (imu_cb_) {
                        imu_cb_(highres_imu.time_usec,
                                highres_imu.xacc, highres_imu.yacc, highres_imu.zacc,
                                highres_imu.xgyro, highres_imu.ygyro, highres_imu.zgyro);
                    }
                    if (mag_cb_) {
                        mag_cb_(highres_imu.time_usec,
                                highres_imu.xmag, highres_imu.ymag, highres_imu.zmag);
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }
    std::cout << "overflow byte" << std::endl;
}
