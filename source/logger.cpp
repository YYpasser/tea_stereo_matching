#include "../include/logger.h"
#include "../include/logger_impl.h"
#include "../include/utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>

// ��̬��Ա��ʼ��
std::shared_ptr<logging::Logger> logging::Logger::instance;
std::mutex logging::Logger::createMtx;

// ����̨�����ʵ��
void logging::ConsoleOutputter::output(const logging::LogMessage& msg)
{
    if (!msg.timestamp.empty()) {
        std::stringstream ss;
        ss << msg.timestamp << " [" << logging::logLevelToString(msg.level) << "] ";
        printf("%s", ss.str().c_str());
        //std::cout << msg.timestamp << " [" << logging::logLevelToString(msg.level) << "] ";
    }
    if (!msg.file.empty() && !msg.function.empty() && msg.line > 0) {
        std::stringstream ss;
        ss << "[" << msg.file << ":" << msg.line << "@" << msg.function << "] ";
        printf("%s", ss.str().c_str());
        //std::cout << "[" << msg.file << ":" << msg.line << "@" << msg.function << "] ";
    }
    if (!msg.content.empty()) {
        printf("%s\n", msg.content.c_str());
        //std::cout << msg.content << std::endl;
    }
    if (!msg.mat.empty()) {
        std::stringstream ss;
        ss << msg.mat << std::endl;
        printf("%s\n", ss.str().c_str());
        //std::cout << msg.mat << std::endl;
    }
}

// �ļ������ʵ��
std::string logger_output_filename;
std::ofstream logger_output_fileStream;
logging::FileOutputter::FileOutputter(const std::string& filename)
{
    logger_output_filename = filename;
    std::cout << "Write log to file: " << filename << std::endl;
    logger_output_fileStream.open(filename, std::ios::app);
    if (!logger_output_fileStream.is_open())
    {
        std::cerr << "Failed to open log file: " << filename << std::endl;
    }
}

logging::FileOutputter::~FileOutputter()
{
    if (logger_output_fileStream.is_open())
    {
        logger_output_fileStream.close();
    }
    std::cout << "Log file save in: " << logger_output_filename << std::endl;
}

void logging::FileOutputter::output(const logging::LogMessage& msg)
{
    if (logger_output_fileStream.is_open())
    {
        if (!msg.timestamp.empty()) {
            logger_output_fileStream << msg.timestamp << " [" << logging::logLevelToString(msg.level) << "] ";
        }
        if (!msg.file.empty() && !msg.function.empty() && msg.line > 0) {
            logger_output_fileStream << "[" << msg.file << ":" << msg.line << "@" << msg.function << "] ";
        }
        if (!msg.content.empty()) {
            logger_output_fileStream << msg.content << std::endl;
        }
        if (!msg.mat.empty()) {
            logger_output_fileStream << msg.mat << std::endl;
        }
    }
}

// ��־��ʽ����ʵ��
std::string logging::LogFormatter::format(const LogMessage& msg)
{
    std::string result = msg.timestamp + " [" + logging::logLevelToString(msg.level) + "] ";
    if (!msg.file.empty() && !msg.function.empty() && msg.line > 0) {
        result += "[" + msg.file + ":" + std::to_string(msg.line) + "@" + msg.function + "] ";
    }
    result += msg.content;
    return result;
}

// ��־��¼��ʵ��
logging::Logger::Logger() : impl(std::make_unique<LoggerImpl>())
{
}

logging::Logger::~Logger()
{
}

// ʹ��˫�ؼ��������ȡ����
std::shared_ptr<logging::Logger> logging::Logger::getInstance()
{
    if (!instance)
    {
        std::lock_guard<std::mutex> lock(createMtx);
        if (!instance)
        {
            // ʹ���Զ���ɾ����ȷ����ȷ�ͷ�
            instance = std::shared_ptr<logging::Logger>(new logging::Logger(),
                [](logging::Logger* ptr) {
                    delete ptr;
                });
        }
    }
    return instance;
}

void logging::Logger::setOutputter(std::shared_ptr<logging::LogOutputter> outputter)
{
    std::lock_guard<std::mutex> lock(this->impl->queueMtx);
    this->impl->outputter = outputter;
}

void logging::Logger::setFormatter(std::shared_ptr<logging::LogFormatter> formatter)
{
    std::lock_guard<std::mutex> lock(this->impl->queueMtx);
    this->impl->formatter = formatter;
}

void logging::Logger::setMinLevel(logging::LogLevel level)
{
    this->impl->minLevel = level;
}

void logging::Logger::log(logging::LogLevel level, const std::string& message,
    const std::string& file, const std::string& function, int line)
{
    // �����־����
    if (level < this->impl->minLevel)
    {
        return;
    }

    logging::LogMessage msg;
    msg.level = level;
    msg.content = message;
    msg.timestamp = utils::getCurrentTime();
    msg.file = file;
    msg.function = function;
    msg.line = line;
    msg.mat = cv::Mat();

    // ����־��Ϣ�������
    {
        std::lock_guard<std::mutex> lock(this->impl->queueMtx);
        this->impl->logQueue.push(msg);
        this->impl->cv.notify_one();
    }
}

void logging::Logger::logMsg(LogLevel level, const std::string& message)
{
    // �����־����
    if (level < this->impl->minLevel)
    {
        return;
    }

    logging::LogMessage msg;
    msg.level = level;
    msg.content = message;
    msg.timestamp = "";
    msg.file = "";
    msg.function = "";
    msg.line = 0;
    msg.mat = cv::Mat();

    // ����־��Ϣ�������
    {
        std::lock_guard<std::mutex> lock(this->impl->queueMtx);
        this->impl->logQueue.push(msg);
        this->impl->cv.notify_one();
    }
}

void logging::Logger::logMat(LogLevel level, const cv::Mat& mat)
{
    // �����־����
    if (level < this->impl->minLevel)
    {
        return;
    }

    logging::LogMessage msg;
    msg.level = level;
    msg.content = "";
    msg.timestamp = "";
    msg.file = "";
    msg.function = "";
    msg.line = 0;
    msg.mat = mat;

    // ����־��Ϣ�������
    {
        std::lock_guard<std::mutex> lock(this->impl->queueMtx);
        this->impl->logQueue.push(msg);
        this->impl->cv.notify_one();
    }
}

void logging::Logger::logMsgMat(LogLevel level, const std::string& message, const cv::Mat& mat)
{
    // �����־����
    if (level < this->impl->minLevel)
    {
        return;
    }

    logging::LogMessage msg;
    msg.level = level;
    msg.content = message;
    msg.timestamp = "";
    msg.file = "";
    msg.function = "";
    msg.line = 0;
    msg.mat = mat;

    // ����־��Ϣ�������
    {
        std::lock_guard<std::mutex> lock(this->impl->queueMtx);
        this->impl->logQueue.push(msg);
        this->impl->cv.notify_one();
    }
}

// LoggerImplʵ��
logging::Logger::LoggerImpl::LoggerImpl() : isRunning(true), minLevel(logging::LogLevel::TEA_INFO)
{
    // Ĭ��ʹ�ÿ���̨�����
    outputter = std::make_shared<logging::ConsoleOutputter>();
    // Ĭ��ʹ�ñ�׼��ʽ����
    formatter = std::make_shared<logging::LogFormatter>();

    // ������־�����߳�
    logThread = std::thread(&logging::Logger::LoggerImpl::processLogs, this);

    auto timestamp = utils::getCurrentTime();
    std::cout << timestamp << " [INFO] Logging system started." << std::endl;
}

logging::Logger::LoggerImpl::~LoggerImpl()
{
    // ֹͣ��־�߳�
    isRunning = false;
    cv.notify_one();

    // �ȴ��߳̽���
    if (logThread.joinable())
    {
        logThread.join();
    }

    // ����ʣ�����־��Ϣ
    std::lock_guard<std::mutex> lock(queueMtx);
    while (!logQueue.empty())
    {
        outputter->output(logQueue.front());
        logQueue.pop();
    }
    auto timestamp = utils::getCurrentTime();
    std::cout << timestamp << " [INFO] Logging system stopped." << std::endl;
}

void logging::Logger::LoggerImpl::processLogs()
{
    while (isRunning)
    {
        std::unique_lock<std::mutex> lock(queueMtx);

        // �ȴ����зǿջ��߳�ֹͣ
        cv.wait(lock, [this]() {
            return !logQueue.empty() || !isRunning;
            });

        // �������д��������־��Ϣ
        while (!logQueue.empty())
        {
            logging::LogMessage msg = logQueue.front();
            logQueue.pop();
            lock.unlock();

            // �����־
            if (outputter)
            {
                outputter->output(msg);
            }

            lock.lock();
        }
    }

    // ����ʣ�����־��ȷ��������־���������
    while (!logQueue.empty())
    {
        logging::LogMessage msg = logQueue.front();
        logQueue.pop();
        if (outputter) {
            outputter->output(msg);
        }
    }
}

void logging::MultiOutputter::addOutputter(std::shared_ptr<LogOutputter> outputter)
{
    if (outputter)
    {
        outputters.push_back(outputter);
    }
}

void logging::MultiOutputter::output(const LogMessage& msg)
{
    for (auto& out : outputters)
    {
        out->output(msg);
    }
}

std::string logging::logLevelToString(LogLevel level)
{
    switch (level)
    {
    case LogLevel::TEA_DEBUG:   return "DEBUG";
    case LogLevel::TEA_INFO:    return "INFO";
    case LogLevel::TEA_WARNING: return "WARNING";
    case LogLevel::TEA_ERROR:   return "ERROR";
    case LogLevel::TEA_FATAL:   return "FATAL";
    default:                return "UNKNOWN";
    }
}
