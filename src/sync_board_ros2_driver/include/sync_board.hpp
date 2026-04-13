#pragma once
// #include "serial_port.hpp"
#include "serial_manager.hpp"

#include "mavlink_handler.hpp"
#include "mavlink_sender.hpp"
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>
#include <condition_variable>
#include "logger.hpp"


struct ImuData {
    uint64_t timestamp_us;
    float ax, ay, az;
    float gx, gy, gz;
};

class SyncBoard {
public:
    SyncBoard();
    ~SyncBoard();

    bool connect(const std::string &port, int baudrate);
    void disconnect();
    void start_recv_message();  // 启动解析线程
    void stop_recv_message();
    void read_system_time();
    void read_camera_trigger();
    void read_log();

    bool pop_timestamp(uint64_t &recv_ts, uint64_t &cam_ts, bool is_blocking = false);
    void read_highres_imu();
    bool getLatestImu(ImuData &data);
    bool getImuData(ImuData &data, bool blocking = false);
    void onImuData(uint64_t ts, float ax, float ay, float az, float gx, float gy, float gz);
    bool isTimesyncOk() const { return timesync_ok_; }

    void upgradeFirmware(const std::string& upgrade_file_path);
    void CamOneShotTrigger();
    void SetCamFrame(int fps);
    void SetCamTrigger(int edge_rising,int trigger, int fps);
    void SetTimeSource(int time_source);

    uint64_t timestamp;
private:
    
    SerialManager serial_;
    MavlinkHandler *mavlink_handler_ = nullptr;

    std::mutex imu_mutex_;
    std::deque<ImuData> imu_buffer_;
    const size_t imu_buffer_max_ = 200;

    std::mutex camera_ts_mutex_;
    std::queue<std::pair<uint64_t, uint64_t>> camera_ts_queue_;
    const size_t MAX_QUEUE_SIZE = 2;

    std::thread recv_thread_;
    std::atomic<bool> running_ = false;
    std::atomic<bool> timesync_ok_ = false;

    std::condition_variable imu_cv_;
    std::condition_variable camera_ts_cv_;



    bool last_sync_status_ = false;
    std::shared_ptr<spdlog::logger> logger_;
};
