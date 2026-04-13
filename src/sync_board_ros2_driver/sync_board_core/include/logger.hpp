#pragma once

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class Logger {
public:
    // 获取指定模块 logger（延迟初始化）
    static std::shared_ptr<spdlog::logger> getLogger(const std::string& moduleName) {
        auto it = loggers_.find(moduleName);
        if (it != loggers_.end()) {
            return it->second;
        }

        // 创建彩色终端 sink
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("[%H:%M:%S] [%^%l%$] [" + moduleName + "] %v");

        // 创建文件 sink（每个模块单独日志文件）
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(moduleName + ".log", true);

        std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};

        auto logger = std::make_shared<spdlog::logger>(moduleName, sinks.begin(), sinks.end());
        logger->set_level(spdlog::level::debug);      // 默认日志等级
        logger->flush_on(spdlog::level::info);        // INFO 及以上自动刷新

        spdlog::register_logger(logger);              // 注册到 spdlog 全局管理器
        loggers_[moduleName] = logger;
        return logger;
    }

    // 设置全局日志等级（可选）
    static void setGlobalLevel(spdlog::level::level_enum level) {
        for (auto& [name, logger] : loggers_) {
            logger->set_level(level);
        }
    }

private:
    static std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> loggers_;
};

// 定义静态成员
inline std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> Logger::loggers_;
