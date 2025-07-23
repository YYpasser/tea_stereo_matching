/*********************************************************************
 * @file   logger.h
 * @brief  ��Ҷ��ѿ��ժ�����Ӿ���־ģ��
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
 * @brief ��־����ö��.
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
 * @brief ��־��Ϣ�ṹ��.
 */
struct LogMessage
{
    LogLevel level;        /*!< ��־���� */
    std::string content;   /*!< ��־���� */
    std::string timestamp; /*!< ʱ��� */
    std::string file;      /*!< �ļ��� */
    std::string function;  /*!< ������ */
    int line;              /*!< �к� */
    cv::Mat mat;           /*!< Mat���� */
};

/**
 * @brief ��־����ӿ� (����չΪ��ͬ���Ŀ�ĵ�).
 */
class LogOutputter
{
public:
    virtual ~LogOutputter() = default;
    virtual void output(const LogMessage& msg) = 0;
};

/**
 * @brief ����̨�����.
 */
class ConsoleOutputter : public LogOutputter
{
public:
    void output(const LogMessage& msg) override;
};

/**
 * @brief �ļ������.
 */
class FileOutputter : public LogOutputter
{
public:
    explicit FileOutputter(const std::string& filename);
    ~FileOutputter() override;
    void output(const LogMessage& msg) override;

};

/**
 * @brief ����ͬʱ����Ŀ�ĵ������־.
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
 * @brief ��־��ʽ���� (���Զ����ʽ).
 */
class LogFormatter
{
public:
    virtual std::string format(const LogMessage& msg);
};

/**
 * @brief ��־��¼�� (����).
 */
class Logger
{
public:
    /**
     * @brief ��ȡ����ʵ��.
     * @return ����ʵ��ָ��
     */
    static std::shared_ptr<Logger> getInstance();

    /**
     * @brief ������־�����.
     * @param [in] outputter ��־�����ָ��
     */
    void setOutputter(std::shared_ptr<LogOutputter> outputter);

    /**
     * @brief ������־��ʽ����.
     * @param [in] formatter ��־��ʽ����ָ��
     */
    void setFormatter(std::shared_ptr<LogFormatter> formatter);

    /**
     * @brief ���������־����.
     * @param [in] level �����־����
     */
    void setMinLevel(LogLevel level);

    /**
     * @brief ��־�ӿ� (��λ����Ϣ).
     * @param [in] level    ��־����
     * @param [in] message  ��־��Ϣ
     * @param [in] file     �ļ���
     * @param [in] function ������
     * @param [in] line     �к�
     */
    void log(LogLevel level, const std::string& message,
        const std::string& file, const std::string& function, int line);

    /**
     * @brief ��־�ӿ� (�������Ϣ).
     * @param [in] level   ��־����
     * @param [in] message ��־��Ϣ
     */
    void logMsg(LogLevel level, const std::string& message);
    /**
     * @brief ��־�ӿ� (���������).
     * @param [in] level ��־����
     * @param [in] mat   ��־����
     */
    void logMat(LogLevel level, const cv::Mat& mat);
    /**
     * @brief ��־�ӿ� (�������Ϣ�;���).
     * @param [in] level   ��־����
     * @param [in] message ��־��Ϣ
     * @param [in] mat     ��־����
     */
    void logMsgMat(LogLevel level, const std::string& message, const cv::Mat& mat);
private:
    Logger();                          /*!< ˽�й��캯�� */
    ~Logger();                         /*!< ˽���������� */
    Logger(const Logger&) = delete;    /*!< ��ֹ���� */
    Logger& operator=(const Logger&) = delete; /*!< ��ֹ��ֵ */

    class LoggerImpl;
    std::shared_ptr<LoggerImpl> impl; /*!< ʵ����ָ�� */

    static std::shared_ptr<Logger> instance;  /*!< ����ʵ�� */
    static std::mutex createMtx;       /*!< ������ */
};

}
