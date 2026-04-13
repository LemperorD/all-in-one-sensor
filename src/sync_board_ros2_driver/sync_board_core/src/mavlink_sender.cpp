#include "mavlink_sender.hpp"
#include <iostream>
#include <chrono>
#include <thread>

MavlinkSender::MavlinkSender(SerialManager &serial) : serial_(serial) {}

void MavlinkSender::sendCameraTrigger(int continuesly, float trigger_interval_ms, int trigger_mode, int trigger_edge)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Prepare the command
    mavlink_command_int_t cmd;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.command = MAV_CMD_DO_SET_CAM_TRIGG_DIST;
    cmd.param1 = continuesly;                   // 是否连续触发，0：默认连续触发（ parm3=1 失效）；
    cmd.param2 = trigger_interval_ms; // Shutter integration time (0 for default)
    cmd.param3 = trigger_mode;                   // 触发模式，1：单次触发（会使param1失效）；
    cmd.param4 = trigger_edge;        // 触发沿，0：上升沿触发，1：下降沿触发；

    // Encode the command
    mavlink_msg_command_int_encode(1, 200, &msg, &cmd); // Use appropriate system/component IDs

    // Send the command
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_.writeData(buf, len);
}

void MavlinkSender::sendTimeSource(int time_source)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // Prepare the command
    mavlink_command_int_t cmd;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.command = 183; //非MAVLINK标准命令，复用
    cmd.param1 = time_source; // 0: boot, 1: network, 2: GPS
    cmd.param2 = 0; // Unused
    cmd.param3 = 0; // Unused
    cmd.param4 = 0; // Unused
    
    // Encode the command
    mavlink_msg_command_int_encode(1, 200, &msg, &cmd); // Use appropriate system/component IDs

    // Send the command
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_.writeData(buf, len);
}

void MavlinkSender::upgradeProcess(const std::string& upgrade_file_path)
{
    // 发送升级命令
    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // Prepare the command
    mavlink_command_int_t cmd;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.command = MAV_CMD_DO_SET_PARAMETER;
    cmd.param1 = 1;                                     // Example parameter index
    cmd.param2 = 1;                                     // Example parameter value
    cmd.param3 = 0;                                     // Unused
    cmd.param4 = 0;                                     // Unused
    mavlink_msg_command_int_encode(1, 200, &msg, &cmd); // Use appropriate system/component IDs
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    serial_.writeData(buf, len);
    std::cout << "[STM32 Upgrade] Sending command, waiting for response..." << std::endl;

    uint8_t c;
    mavlink_message_t mavlink_msg;
    mavlink_status_t mavlink_status;
    while(true)
    {
        if (serial_.readByte(c) <= 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavlink_msg, &mavlink_status))
        {
            if (mavlink_msg.msgid == MAVLINK_MSG_ID_STATUSTEXT)
            {
                mavlink_statustext_t statustext;
                mavlink_msg_statustext_decode(&mavlink_msg, &statustext);
                std::string status_text(statustext.text);

                size_t newline_pos = status_text.find('\n');
                if (newline_pos != std::string::npos)
                    status_text = status_text.substr(0, newline_pos);

                std::cout << "[Status] " << status_text << std::endl;

                if (status_text == "Jump to bootloader")
                {
                    std::cout << "[STM32 Upgrade] Jump to bootloader detected." << std::endl;
                    serial_.closePort();

                    std::this_thread::sleep_for(std::chrono::seconds(5));

                    std::string dfu_cmd = "dfu-util -a 0 -s 0x08000000:leave -D " + upgrade_file_path;
                    int ret_code = std::system(dfu_cmd.c_str());
                    if (ret_code == 0)
                    {
                        std::cout << "[STM32 Upgrade] Firmware upgrade completed successfully." << std::endl;
                    }
                    else
                    {
                        std::cerr << "[STM32 Upgrade] Firmware upgrade failed with return code: " << ret_code << std::endl;
                    }

                    std::cout << "[STM32 Upgrade] Exiting..." << std::endl;
                    return;
                }
            }
        }
    }
}