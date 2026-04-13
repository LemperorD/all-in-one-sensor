#pragma once
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <iostream>

class SerialManager {
public:
    SerialManager(): io_(), serial_(io_),
          running_(false), reconnect_interval_ms_(2000) {}
    SerialManager(const std::string& port, unsigned int baud_rate)
        : io_(), serial_(io_), port_(port), baud_rate_(baud_rate),
          running_(false), reconnect_interval_ms_(2000) {}

    ~SerialManager() {
        stop();
    }
    bool openPort(const std::string& port, unsigned int baud_rate) {
        port_ = port;
        baud_rate_ = baud_rate;
        start();
        return true;
    }
    void start() {
        running_ = true;
        sender_thread_ = std::thread(&SerialManager::senderLoop, this);
    }

    void stop() {
        running_ = false;
        data_cond_.notify_all();
        if (sender_thread_.joinable())
            sender_thread_.join();
        close();
    }
    bool writeData(const uint8_t *data, size_t len) {
        std::vector<uint8_t> vec_data(data, data + len);
        writeData(vec_data);
        return true;
    }

    void writeData(const std::vector<uint8_t>& data) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            send_queue_.push(data);
        }
        data_cond_.notify_one();
    }

    std::vector<uint8_t> read(size_t num_bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<uint8_t> buf(num_bytes);
        if (!serial_.is_open()) return {};
        boost::system::error_code ec;
        size_t len = boost::asio::read(serial_, boost::asio::buffer(buf), boost::asio::transfer_exactly(num_bytes), ec);
        if (ec) return {};
        buf.resize(len);
        return buf;
    }
    int readByte(uint8_t &byte) {
        boost::system::error_code ec;
        size_t len = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!serial_.is_open()) 
            {
                std::cout << "Serial port not open\n";
                return 0;
            }
            
            len = boost::asio::read(serial_, boost::asio::buffer(&byte, 1), ec);
        }

        if (ec) 
        {
            std::cout << ec.message() << std::endl;
            close();
            while (running_ && !open()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms_));
                std::cout << "try again" << std::endl;
            }
            std::cout << "reconnect success" << std::endl;

            return 0;
        }
        if(len == 0)return 0;
        return 1;
    }

    bool isOpen() {
        std::lock_guard<std::mutex> lock(mutex_);
        return serial_.is_open();
    }
    void closePort(){
        close();
    }

private:
    void senderLoop() {

        while (running_) {
            if (!isOpen()) {
                while (running_ && !open()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms_));
                }
            }
            std::vector<uint8_t> data;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                data_cond_.wait(lock, [this] { return !send_queue_.empty() || !running_; });
                if (!running_) break;
                data = send_queue_.front();
                send_queue_.pop();
            }
            if (!write(data)) {
                close();
            }
        }
    }

    bool open() {
        std::cout << "Opening port " << port_ << " at baud rate " << baud_rate_ << std::endl;
        std::lock_guard<std::mutex> lock(mutex_);
        boost::system::error_code ec;
        serial_.open(port_, ec);
        if (ec)
        {
            std::cerr << "Failed to open port " << port_ << ": " << ec.message() << std::endl;
            return false;
        } 
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (serial_.is_open())
            serial_.close();
    }

    bool write(const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!serial_.is_open()) return false;
        boost::system::error_code ec;
        boost::asio::write(serial_, boost::asio::buffer(data), ec);
        return !ec;
    }

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::string port_;
    unsigned int baud_rate_;
    std::mutex mutex_;

    std::thread sender_thread_;
    std::atomic<bool> running_;
    std::queue<std::vector<uint8_t>> send_queue_;
    std::mutex queue_mutex_;
    std::condition_variable data_cond_;
    int reconnect_interval_ms_;
};