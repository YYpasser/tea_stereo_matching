/*********************************************************************
 * @file   logger_impl.h
 * @brief  茶叶嫩芽采摘立体视觉日志模块实现类
 * @author Qianyao Zhuang
 * @date   May 2025
 *********************************************************************/
#pragma once
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <opencv2/core.hpp>
#include "./logger.h"

namespace logging
{
    class Logger::LoggerImpl
    {
    public:
        LoggerImpl();
        ~LoggerImpl();

        // 日志线程处理函数
        void processLogs();

        // 消息队列相关
        std::queue<LogMessage> logQueue;   /*!< 日志消息队列 */
        std::mutex queueMtx;               /*!< 队列锁 */
        std::condition_variable cv;        /*!< 条件变量, 用于通知日志线程 */
        std::thread logThread;             /*!< 日志处理线程 */
        std::atomic<bool> isRunning;       /*!< 线程运行标志 */

        // 输出器和格式化器
        std::shared_ptr<LogOutputter> outputter;
        std::shared_ptr<LogFormatter> formatter;

        // 最低日志级别
        LogLevel minLevel;
    };
}