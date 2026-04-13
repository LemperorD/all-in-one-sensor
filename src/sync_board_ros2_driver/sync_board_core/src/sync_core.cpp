#include "sync_core.hpp"
#include "serial_device.hpp"


int main(int argc, char* argv[])
{
    auto console = Logger::getLogger("sync_core");
    signal(SIGPIPE, SIG_IGN);
    cxxopts::Options options("Sync Core", "Sync Server");
    options.add_options()
        ("a,address", "Server address", cxxopts::value<std::string>()->default_value("127.0.0.1"))
        ("p,port", "Base port", cxxopts::value<int>()->default_value("12080"))
        ("h,help", "Print help");
    auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
        std::cout << options.help() << std::endl;
        return 0;
    }
    serial_device serial;
    std::vector<std::string> devices = serial.list_devices("ttyEmnvSensorBridge_");
    for (const auto& device : devices) {
        std::cout << device << std::endl;
    }
    if(devices.empty()) {

        console->error("No sync board device found!");
        return -1;
    }

    SyncServer manager(devices[0], result["port"].as<int>(), 10);
    sync_proto::LogLevel log_level = sync_proto::LogLevel::DEBUG;
    static uint64_t last_time = 0;

    std::thread imu_thread([&manager]()
                           {
        auto imu_log = Logger::getLogger("imu");
        while (true) {
            ImuData imu;
            static int imu_count = 0;
            if (manager.board.getImuData(imu,true))
            {
                imu_count++;
                uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
                if (last_time == 0) last_time = now;
                if (now - last_time >= 5000) // 每5秒统计一次
                {
                    imu_log->info("Frequency: {} Hz", imu_count / 5);
                    // std::cout << "[IMU] Send Frequency: " << imu_count << " Hz" << std::endl;
                    imu_count = 0;
                    last_time = now;
                }
                sync_proto::IMU_RAW imu_msg;
                imu_msg.set_time_us(imu.timestamp_us);
                imu_msg.set_ax(imu.ax);
                imu_msg.set_ay(imu.ay);
                imu_msg.set_az(imu.az);
                imu_msg.set_gx(imu.gx);
                imu_msg.set_gy(imu.gy);
                imu_msg.set_gz(imu.gz);
                // Serialize to string
                std::string serialized_data;

                if (manager.SerializeMessage(sync_proto::MessageID::MSG_IMU_RAW, imu_msg, serialized_data))
                {
                manager.send_to_queues(serialized_data);
                }
            }
        } });
    imu_thread.detach();

    std::thread trigger_thread([&manager]()
                               {
        while (true)
        {
            uint64_t cam_ts;
            uint64_t recv_ts;
            if (manager.board.pop_timestamp(recv_ts, cam_ts,true))
            {
                sync_proto::TRIGGER trigger_msg;
                trigger_msg.set_trigger_time_us(cam_ts);
                trigger_msg.set_recv_trigger_time_us(recv_ts);
                // std::cout << "[TRIGGER] Camera Timestamp: " << cam_ts << " us, "
                //           << "Received Timestamp: " << recv_ts << " us" << std::endl;
                std::string serialized_data;
                if (manager.SerializeMessage(sync_proto::MessageID::MSG_TRIGGER, trigger_msg, serialized_data))
                    manager.send_to_queues(serialized_data);
            }
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } });
    trigger_thread.detach();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "Server exiting...\n";
    return 0;
}
