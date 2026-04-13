#pragma once
#include <iostream>
#include <thread>
#include <unordered_map>
#include <deque>
#include <set>
#include <mutex>
#include <atomic>
#include <vector>
#include <signal.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "sync_board.hpp"
#include "sync.pb.h"
#include <condition_variable>
#include <netinet/tcp.h>
#include <cxxopts.hpp>

#define MAGIC_CODE 0xA1CA

class SyncServer
{
public:
    SyncServer(std::string device_path, int base_port, int max_ports)
        : base_port_(base_port), max_ports_(max_ports)
    {
        logger_ = Logger::getLogger("SyncServer");
        if (!board.connect(device_path, 921600))
        {
            logger_->error("Failed to connect");
            return;
        }
        board.read_highres_imu();    // 注册IMU回调
        board.read_system_time();    // 注册系统时间回调
        board.read_camera_trigger(); // 注册相机触发回调
        board.start_recv_message();  // 启动消息接收线程
        board.read_log();  // 注册日志回调

        // 初始化端口池
        for (int i = 1; i <= max_ports_; ++i)
        {
            available_ports_.push_back(base_port_ + i);
            add_queue(base_port_ + i);
        }

        // 启动 base_port 的监听线程
        start_alloc_port_thread(base_port_);
    }
    ~SyncServer()
    {
    }

    // 分配端口
    int allocate_port()
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (available_ports_.empty())
            return -1;

        int port = available_ports_.front();
        available_ports_.pop_front();
        allocated_ports_.insert(port);
        return port;
    }

    // 释放端口
    void release_port(int port)
    {
        std::lock_guard<std::mutex> lock(mu_);
        auto it = allocated_ports_.find(port);
        if (it != allocated_ports_.end())
        {
            allocated_ports_.erase(it);
            available_ports_.push_back(port);
        }
    }
    void add_queue(const int client_port)
    {
        std::lock_guard<std::mutex> lock(queues_mu);
        queues.emplace_back(client_port, std::deque<std::string>());
        conn_status.emplace_back(client_port, false);
    }
    void set_conn_status(const int client_port, bool status)
    {
        // 不用加锁，就是个bool变量
        for (auto &conn : conn_status)
        {
            if (conn.first == client_port)
            {
                conn.second = status;
                break;
            }
        }
    }
    int get_port_index(const int client_port)
    {
        std::lock_guard<std::mutex> lock(queues_mu);
        if (queues.empty())
            return -1;
        for (size_t i = 0; i < queues.size(); ++i)
        {
            if (queues[i].first == client_port)
            {
                return i;
            }
        }
    }
    void send_to_queues(const std::string &data)
    {
        std::lock_guard<std::mutex> lock(queues_mu);
        for (size_t i = 0; i < queues.size(); ++i)
        {
            if (conn_status[i].second) // 只有连接状态为true才推送
            {
                queues[i].second.push_back(data);
                sender_queues_cv.notify_one();
            }
        }
    }
    template <typename T>
    bool SerializeMessage(sync_proto::MessageID id, const T &msg, std::string &out)
    {
        std::string payload;
        if (!msg.SerializeToString(&payload))
            return false;

        out.clear();
        out.reserve(payload.size() + 6);

        uint16_t magic = htons(MAGIC_CODE);
        uint16_t len = htons(payload.size());
        uint16_t id_u16 = htons(static_cast<uint16_t>(id));

        out.append(reinterpret_cast<char *>(&magic), sizeof(magic));
        out.append(reinterpret_cast<char *>(&len), sizeof(len));
        out.append(reinterpret_cast<char *>(&id_u16), sizeof(id_u16));
        out.append(payload);

        return true;
    }
    int configParse(const std::string& data, sync_proto::CfgPropID& prop_id, bool& bool_val, int32_t& int_val, float& float_val)
    {
        // 遍历 data 寻找 MAGIC_CODE
        size_t pos = 0;
        while (pos + 6 <= data.size()) {
            uint16_t magic = ntohs(*reinterpret_cast<const uint16_t*>(data.data() + pos));
            if (magic == MAGIC_CODE) {
                uint16_t len = ntohs(*reinterpret_cast<const uint16_t*>(data.data() + pos + 2));
                uint16_t id_u16 = ntohs(*reinterpret_cast<const uint16_t*>(data.data() + pos + 4));
                if (id_u16 != static_cast<uint16_t>(sync_proto::MessageID::MSG_CONFIG)) {
                    pos += 1;
                    continue;
                }
                if (pos + 6 + len > data.size()) return -1;
                std::string payload = data.substr(pos + 6, len);
                sync_proto::ConfigItem config_msg;
                if (!config_msg.ParseFromString(payload)) return -1;
                prop_id = static_cast<sync_proto::CfgPropID>(config_msg.prop_id());
                bool_val = config_msg.bool_value();
                int_val = config_msg.int_value();
                float_val = config_msg.float_value();
                return pos + 6 + len;
            }
            pos += 1;
        }
        return -1;
    }
    void recvCfgThreadStart(int sockfd)
    {
        std::thread([this, sockfd]()
                    {
            auto cfg_log = Logger::getLogger("CFG SET");
            std::string total_data;
            sync_proto::CfgPropID prop_id;
            bool bool_val;
            int32_t int_val;
            float float_val;
            while (true) {
                if(sockfd < 0) break;
                char buffer[1024];
                int n = recv(sockfd, buffer, sizeof(buffer) - 1, 0);
                if (n <= 0) {
                    break;
                }
                total_data.append(buffer, n);
                int pos = configParse(total_data, prop_id, bool_val, int_val, float_val);
                if(pos > 0)
                {
                   switch (prop_id)
                   {
                   case sync_proto::CfgPropID::CFG_INT_TRIGGER_FREQ:
                        cfg_log->info("CFG_INT_TRIGGER_FREQ: {}", int_val);
                        board.SetCamTrigger(-1,-1,int_val);
                        break;
                   case sync_proto::CfgPropID::CFG_BOOL_TRIGGER_EDGE:
                        cfg_log->info("CFG_BOOL_TRIGGER_EDGE: {}", bool_val);
                        board.SetCamTrigger(bool_val,-1,-1);
                        break;
                    case sync_proto::CfgPropID::CFG_BOOL_TRIGGER_ONCE:
                        cfg_log->info(" CFG_BOOL_TRIGGER_MODE_CONTINUOUS: {}", bool_val);
                        board.SetCamTrigger(-1,1,0);
                        break;
                   default:
                    break;
                   }
                }
            }
            logger_->info("[CFG Stopped] TCP port {}", base_port_ + max_ports_ + 1);
        })
            .detach();
    }



private:
    // 启动监听线程
    void start_alloc_port_thread(int port)
    {
        std::atomic<bool> *flag = new std::atomic<bool>(true);

        listen_flags_[port] = flag;

        listen_threads_[port] = std::thread([this, port, flag]()
                                            {
            int sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd < 0) {
                logger_->error("Socket creation failed on port {}", port);
                return;
            }

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = htons(port);

            int opt = 1;
            setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
                logger_->error("Bind failed on port {}", port);
                close(sockfd);
                return;
            }

            if (listen(sockfd, 5) < 0) {
                logger_->error("Listen failed on port {}", port);
                close(sockfd);
                return;
            }

            logger_->info("[Listening] TCP port {}", port);

            while (flag->load()) {
                int client = accept(sockfd, nullptr, nullptr);
                if (client >= 0) {
                    logger_->info("[Connect] port {}", port);
                    int port = allocate_port();
                    if (port != -1)
                        logger_->info("Allocated port: {}", port);
                    else
                        logger_->error("No available ports.");
                    //开一个线程 新开端口用于传输数据流
                    std::thread(&SyncServer::start_data_stream_thread, this, port).detach();
                    send(client, &port, sizeof(port), 0);
                    close(client);
                    
                }
            }

            close(sockfd);
            sockfd = -1;
            logger_->info("[Stopped] TCP port {}", port);
        });
        listen_threads_[port].detach();
    }
    void start_data_stream_thread(int port)
    {
        int sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0)
        {
            logger_->error("Socket creation failed on data port {}", port);
            return;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        int opt = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        int flag = 1;
        if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0)
        {
            logger_->error("Failed to set TCP_NODELAY on data port {}", port);
        }
        if (bind(sockfd, (sockaddr *)&addr, sizeof(addr)) < 0)
        {
            logger_->error("Bind failed on data port {}", port);
            close(sockfd);
            return;
        }

        if (listen(sockfd, 1) < 0)
        {
            logger_->error("Listen failed on data port {}", port);
            close(sockfd);
            return;
        }

        logger_->info("Listening on port {}", port);
        int data_client = accept(sockfd, nullptr, nullptr);
        int idx = get_port_index(port);

        recvCfgThreadStart(data_client);
        if (data_client >= 0)
        {
            logger_->info("Client connected on port {}", port);
            set_conn_status(port, true);
            // logger_->info("Queue index for port {} is {}", port, idx);
            // 发送数据
            while (conn_status[idx].second)
            {
                {
                    // 等待队列有数据或者连接断开
                    std::unique_lock<std::mutex> lk(queues_mu);
                    sender_queues_cv.wait(lk, [&]
                                          { return !queues[idx].second.empty() || !conn_status[idx].second; });
                    // if (!conn_status[idx].second) break;
                    while (!queues[idx].second.empty() && conn_status[idx].second)
                    {
                        if (queues[idx].second.size() > 10)
                        {
                            logger_->warn("[Data Server] Warning: Queue size for port {} is {}", port, queues[idx].second.size());
                        }
                        std::string data = queues[idx].second.front();
                        queues[idx].second.pop_front();
                        try
                        {
                            int ret = send(data_client, data.c_str(), data.size(), 0);
                            if (ret <= 0)
                            {
                                logger_->error("[Data Server] Send failed on port {}", port);
                                set_conn_status(port, false);
                                continue;
                            }
                        }
                        catch (...)
                        {
                            logger_->error("[Data Server] Exception occurred during send on port {}", port);
                            break;
                        }
                    }
                }
                // std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            close(data_client);
        }
        close(sockfd);
        sockfd = -1;
        logger_->info("[Data Server] Closed connection on port {}", port);

        release_port(port);
    }

private:
    int base_port_;
    int max_ports_;
    std::deque<int> available_ports_;
    std::set<int> allocated_ports_;
    std::mutex mu_;

    std::unordered_map<int, std::thread> listen_threads_;
    std::unordered_map<int, std::atomic<bool> *> listen_flags_;
    std::vector<std::pair<int, std::deque<std::string>>> queues;
    std::vector<std::pair<int, bool>> conn_status;

    std::mutex queues_mu;
    std::condition_variable sender_queues_cv;

    std::shared_ptr<spdlog::logger> logger_;


public:
    SyncBoard board;
};
