#include "sync_board.hpp"
#include <iostream>
#include <chrono>
#include <thread>


SyncBoard::SyncBoard() {
    logger_ = Logger::getLogger("SyncBoard");
}
SyncBoard::~SyncBoard() { stop_recv_message(); disconnect(); }

bool SyncBoard::connect(const std::string &port, int baudrate) {
    if (!serial_.openPort(port, baudrate)) return false;

    mavlink_handler_ = new MavlinkHandler(serial_);
    mavlink_handler_->setHeartbeatCallback([this](bool ok) {
        timesync_ok_ = ok;
        if (ok)
        {
            if(!last_sync_status_)
                logger_->info("[TIMESYNC] SUCCESS");
        } else logger_->info("[TIMESYNC] IN PROGRESS");
        last_sync_status_ = ok;

    });

    return true;
}

void SyncBoard::start_recv_message() {
    running_ = true;
    recv_thread_ = std::thread([this]() {
        while (running_) {
            mavlink_handler_->processIncoming();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void SyncBoard::read_system_time(){
    mavlink_handler_->setSystemtimeCallback([this](uint64_t time_unix_usec){
        timestamp = time_unix_usec;
        // std::cout << "[System Time Callback] MAVLink time: " << time_unix_usec << std::endl;
    });
}

void SyncBoard::read_camera_trigger(){
    mavlink_handler_->setCameratriggerCallback([this](uint64_t recv_host_usec ,uint64_t time_usec){
        // std::cout << "[Camera Trigger] Camera triggered at time: " << time_usec << " us" << std::endl;
        std::lock_guard<std::mutex> lock(camera_ts_mutex_);

        if(camera_ts_queue_.size() >= MAX_QUEUE_SIZE) {
            camera_ts_queue_.pop();
        }
        camera_ts_queue_.push(std::pair(recv_host_usec ,time_usec));
        camera_ts_cv_.notify_one();
    });
}


void SyncBoard::read_log(){
    mavlink_handler_->setLogCallback([this](uint8_t severity, const std::string &text){
        if(severity <= 3) { // Error or Critical
            logger_->error("LOG: {}", text);
        } else if (severity == 4) { // Warning
            logger_->warn("LOG: {}", text);
        } else if (severity == 6) { // Info
            logger_->info("LOG: {}", text);
        } else { // Debug or lower
            logger_->debug("LOG: {}", text);
        }
        // std::cout << "[LOG] Severity: " << static_cast<int>(severity) << ", Message: " << text << std::endl;
    });
}


bool SyncBoard::pop_timestamp(uint64_t &recv_ts, uint64_t &cam_ts,bool is_blocking){
    std::unique_lock<std::mutex> lock(camera_ts_mutex_);

    if(camera_ts_queue_.empty()) {
        if(is_blocking) {
            camera_ts_cv_.wait(lock, [&]{ return !camera_ts_queue_.empty(); });
        } else {
            return false;
        }
    }
    recv_ts = camera_ts_queue_.front().first;
    cam_ts = camera_ts_queue_.front().second;
    camera_ts_queue_.pop();
    return true;
}

void SyncBoard::read_highres_imu(){
    mavlink_handler_->setImuCallback([this](uint64_t ts,
                                        float ax, float ay, float az,
                                        float gx, float gy, float gz){
        this->onImuData(ts, ax, ay, az, gx, gy, gz);
    });
}

void SyncBoard::onImuData(uint64_t ts, float ax, float ay, float az, float gx, float gy, float gz) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (imu_buffer_.size() >= imu_buffer_max_) {
        imu_buffer_.pop_front();  // 移除最旧的数据
    }
    imu_buffer_.push_back({ts, ax, ay, az, gx, gy, gz});
    imu_cv_.notify_one();
}

bool SyncBoard::getLatestImu(ImuData &data) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (imu_buffer_.empty()) return false;
    data = imu_buffer_.back();
    return true;
}
bool SyncBoard::getImuData(ImuData &data, bool blocking) {
    std::unique_lock<std::mutex> lock(imu_mutex_);
    if (blocking) {
        imu_cv_.wait(lock, [&]{ return !imu_buffer_.empty(); });
    }
    if (imu_buffer_.empty()) return false;
    data = imu_buffer_.front();
    imu_buffer_.pop_front();
    return true;
}

void SyncBoard::CamOneShotTrigger()
{
    MavlinkSender sender(serial_);
    sender.sendCameraTrigger(1, 0.0f, 1, 0);
}

void SyncBoard::SetCamFrame(int fps)
{
    MavlinkSender sender(serial_);
    float interval_ms = 1000.0f / fps;
    sender.sendCameraTrigger(0, interval_ms, 0, 0);
}

void SyncBoard::SetCamTrigger(int edge_rising,int trigger, int fps)
{
    // 忽略<0的配置项
    // continuously_mode 0 连续 1 单次 这是很奇怪的
    MavlinkSender sender(serial_);
    float interval_ms;
    int continuously_mode;
    if(fps > 0)
    {
        interval_ms = 1000.0f / fps;
        continuously_mode = 0;
    } 
    else if(fps == 0) 
    {
        interval_ms = -1.0f;
        continuously_mode = 1;
    }

    if(trigger == 1)
    {
        continuously_mode = 0; // 单次模式
        interval_ms = -1.0f;
        trigger = 1;
    }
    sender.sendCameraTrigger(continuously_mode, interval_ms, trigger, edge_rising);
}

void SyncBoard::SetTimeSource(int time_source)
{
    MavlinkSender sender(serial_); 
    sender.sendTimeSource(time_source);
}
void SyncBoard::upgradeFirmware(const std::string& upgrade_file_path)
{
    MavlinkSender sender(serial_);
    sender.upgradeProcess(upgrade_file_path);
}

void SyncBoard::disconnect() {
    if (serial_.isOpen()) serial_.closePort();
    if (mavlink_handler_) {
        delete mavlink_handler_;
        mavlink_handler_ = nullptr;
    }
}

void SyncBoard::stop_recv_message() {
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
}
