/*!
 * @file Log.cpp
 * @brief 日志系统初始化的实现。
 */

#include "Utilities/Log.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>

void initLogger() {
  try {
    // 创建一个带颜色的控制台 sink
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    // 设置日志格式：
    // [%H:%M:%S.%e] 带毫秒的时间
    // [%^%l%$]      带颜色的日志级别（开始颜色 %^，结束颜色 %$）
    // %v            实际消息
    console_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

    // 创建名为 "console" 的 logger
    auto logger = std::make_shared<spdlog::logger>("console", console_sink);

    // 设置为全局宏使用的默认 logger
    spdlog::set_default_logger(logger);

    // 设置默认日志级别为 INFO
    // TODO: 考虑从配置文件读取此设置
    spdlog::set_level(spdlog::level::info);

    // 在 INFO 级别刷新，以确保日志立即写入
    spdlog::flush_on(spdlog::level::info);

    LOG_INFO("Logger initialized successfully with color support.");
  } catch (const spdlog::spdlog_ex& ex) {
    printf("Log initialization failed: %s\n", ex.what());
  }
}
