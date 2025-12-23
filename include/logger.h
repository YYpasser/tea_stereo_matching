/*********************************************************************
 * @file   logger.h
 * @brief  茶叶嫩芽采摘立体视觉日志模块
 * @author Qianyao Zhuang
 * @date   May 2025
 *********************************************************************/
#pragma once
#include <string>
#include <vector>
#include <memory>
#include <opencv2/core.hpp>

#define GET_FILENAME(path) (strrchr(path, '/') ? strrchr(path, '/') + 1 :\
 strrchr(path, '\\') ? strrchr(path, '\\') + 1 : path)

#define LOG_DEBUG(message) \
    logging::Logger::getInstance()->log(logging::LogLevel::TEA_DEBUG, message, GET_FILENAME(__FILE__), __FUNCTION__, __LINE__)

#define LOG_INFO(message) \
    logging::Logger::getInstance()->log(logging::LogLevel::TEA_INFO, message, GET_FILENAME(__FILE__), __FUNCTION__, __LINE__)

#define LOG_WARNING(message) \
    logging::Logger::getInstance()->log(logging::LogLevel::TEA_WARNING, message, GET_FILENAME(__FILE__), __FUNCTION__, __LINE__)

#define LOG_ERROR(message) \
    logging::Logger::getInstance()->log(logging::LogLevel::TEA_ERROR, message, GET_FILENAME(__FILE__), __FUNCTION__, __LINE__)

#define LOG_FATAL(message) \
    logging::Logger::getInstance()->log(logging::LogLevel::TEA_FATAL, message, GET_FILENAME(__FILE__), __FUNCTION__, __LINE__)

#define LOG_INFO_MSG(message) \
    logging::Logger::getInstance()->logMsg(logging::LogLevel::TEA_INFO, message)

#define LOG_INFO_MAT(mat) \
    logging::Logger::getInstance()->logMat(logging::LogLevel::TEA_INFO, mat)

#define LOG_INFO_MSG_MAT(message, mat) \
    logging::Logger::getInstance()->logMsgMat(logging::LogLevel::TEA_INFO, message, mat)

namespace logging
{

    /**
     * @brief 日志级别枚举.
     */
    enum class LogLevel
    {
        TEA_DEBUG,
        TEA_INFO,
        TEA_WARNING,
        TEA_ERROR,
        TEA_FATAL
    };
    inline std::string logLevelToString(LogLevel level);

    /**
     * @brief 日志信息结构体.
     */
    struct LogMessage
    {
        LogLevel level;        /*!< 日志级别 */
        std::string content;   /*!< 日志内容 */
        std::string timestamp; /*!< 时间戳 */
        std::string file;      /*!< 文件名 */
        std::string function;  /*!< 函数名 */
        int line;              /*!< 行号 */
        cv::Mat mat;           /*!< Mat对象 */
    };

    /**
     * @brief 日志输出接口 (可扩展为不同输出目的地).
     */
    class LogOutputter
    {
    public:
        virtual ~LogOutputter() = default;
        virtual void output(const LogMessage& msg) = 0;
    };

    /**
     * @brief 控制台输出器.
     */
    class ConsoleOutputter : public LogOutputter
    {
    public:
        void output(const LogMessage& msg) override;
    };

    /**
     * @brief 文件输出器.
     */
    class FileOutputter : public LogOutputter
    {
    public:
        explicit FileOutputter(const std::string& filename);
        ~FileOutputter() override;
        void output(const LogMessage& msg) override;

    };

    /**
     * @brief 可以同时向多个目的地输出日志.
     */
    class MultiOutputter : public LogOutputter
    {
    public:
        void addOutputter(std::shared_ptr<LogOutputter> outputter);
        void output(const LogMessage& msg) override;

    private:
        std::vector<std::shared_ptr<LogOutputter>> outputters;
    };

    /**
     * @brief 日志格式化器 (可自定义格式).
     */
    class LogFormatter
    {
    public:
        virtual std::string format(const LogMessage& msg);
    };

    /**
     * @brief 日志记录器 (单例).
     */
    class Logger
    {
    public:
        /**
         * @brief 获取单例实例.
         * @return 单例实例指针
         */
        static std::shared_ptr<Logger> getInstance();

        /**
         * @brief 设置日志输出器.
         * @param [in] outputter 日志输出器指针
         */
        void setOutputter(std::shared_ptr<LogOutputter> outputter);

        /**
         * @brief 设置日志格式化器.
         * @param [in] formatter 日志格式化器指针
         */
        void setFormatter(std::shared_ptr<LogFormatter> formatter);

        /**
         * @brief 设置最低日志级别.
         * @param [in] level 最低日志级别
         */
        void setMinLevel(LogLevel level);

        /**
         * @brief 日志接口 (带位置信息).
         * @param [in] level    日志级别
         * @param [in] message  日志信息
         * @param [in] file     文件名
         * @param [in] function 函数名
         * @param [in] line     行号
         */
        void log(LogLevel level, const std::string& message,
            const std::string& file, const std::string& function, int line);

        /**
         * @brief 日志接口 (仅输出消息).
         * @param [in] level   日志级别
         * @param [in] message 日志信息
         */
        void logMsg(LogLevel level, const std::string& message);
        /**
         * @brief 日志接口 (仅输出矩阵).
         * @param [in] level 日志级别
         * @param [in] mat   日志矩阵
         */
        void logMat(LogLevel level, const cv::Mat& mat);
        /**
         * @brief 日志接口 (仅输出消息和矩阵).
         * @param [in] level   日志级别
         * @param [in] message 日志信息
         * @param [in] mat     日志矩阵
         */
        void logMsgMat(LogLevel level, const std::string& message, const cv::Mat& mat);
    private:
        Logger();                          /*!< 私有构造函数 */
        ~Logger();                         /*!< 私有析构函数 */
        Logger(const Logger&) = delete;    /*!< 禁止复制 */
        Logger& operator=(const Logger&) = delete; /*!< 禁止赋值 */

        class LoggerImpl;
        std::shared_ptr<LoggerImpl> impl; /*!< 实现类指针 */

        static std::shared_ptr<Logger> instance;  /*!< 单例实例 */
        static std::mutex createMtx;       /*!< 单例锁 */
    };

}
