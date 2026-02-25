/*!
 * @file Log.h
 * @brief 提供项目的日志宏和初始化功能。
 *
 * 本文件封装了 spdlog 功能，为 Cheetah Software 项目提供一致的日志接口。
 * 它定义了不同日志级别的宏，并声明了初始化函数。
 */

#ifndef PROJECT_LOG_H
#define PROJECT_LOG_H

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h> // 支持自定义类型输出

/*!
 * @brief 初始化日志系统。
 *
 * 设置带有颜色控制台 sink 和特定格式的默认 logger。
 * 应在程序开始时（例如在 main 中）调用一次。
 */
void initLogger();

// 使用默认 spdlog logger 的日志宏

/*! @brief 记录 trace 消息。 */
#define LOG_TRACE(...) spdlog::trace(__VA_ARGS__)

/*! @brief 记录 debug 消息。 */
#define LOG_DEBUG(...) spdlog::debug(__VA_ARGS__)

/*! @brief 记录 info 消息。 */
#define LOG_INFO(...)  spdlog::info(__VA_ARGS__)

/*! @brief 记录 warning 消息。 */
#define LOG_WARN(...)  spdlog::warn(__VA_ARGS__)

/*! @brief 记录 error 消息。 */
#define LOG_ERROR(...) spdlog::error(__VA_ARGS__)

/*! @brief 记录 critical error 消息。 */
#define LOG_CRITICAL(...) spdlog::critical(__VA_ARGS__)

#endif // PROJECT_LOG_H
