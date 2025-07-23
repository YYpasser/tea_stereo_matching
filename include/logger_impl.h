/*********************************************************************
 * @file   logger_impl.h
 * @brief  ��Ҷ��ѿ��ժ�����Ӿ���־ģ��ʵ����
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

        // ��־�̴߳�����
        void processLogs();

        // ��Ϣ�������
        std::queue<LogMessage> logQueue;   /*!< ��־��Ϣ���� */
        std::mutex queueMtx;               /*!< ������ */
        std::condition_variable cv;        /*!< ��������, ����֪ͨ��־�߳� */
        std::thread logThread;             /*!< ��־�����߳� */
        std::atomic<bool> isRunning;       /*!< �߳����б�־ */

        // ������͸�ʽ����
        std::shared_ptr<LogOutputter> outputter;
        std::shared_ptr<LogFormatter> formatter;

        // �����־����
        LogLevel minLevel;
    };
}