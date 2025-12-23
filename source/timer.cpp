#include "../include/timer.h"
#include "../include/logger.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <sstream>
#include <chrono>
#include <iostream>


class Timer::TimerImpl
{
public:
    TimerImpl(const std::string& timerName, Callback callback, TimeInterval interval, bool singleShot) :
        m_timerName(timerName),
        m_callback(std::move(callback)),
        m_interval(interval),
        m_singleShot(singleShot),
        m_isRunning(false) { }
    ~TimerImpl() 
    {
        stop();
        if (this->workerThread.joinable())
        {
            this->workerThread.join();
            LOG_INFO(this->m_timerName + ": 工作线程已正常退出.");
        }
    }

    void start()
    {
        if (this->m_isRunning.exchange(true))
        {
            LOG_WARNING(this->m_timerName + ": 已处于运行状态, 无需重复启动.");
            return;
        }

        this->workerThread = std::thread(&TimerImpl::workerLoop, this);
    }
    void stop()
    {
        if (!this->m_isRunning.load())
        {
            return;
        }
        this->m_isRunning.store(false);
        this->m_cv.notify_one();
    }
    void reset()
    {
        LOG_INFO(this->m_timerName + ": 开始重置定时器.");
        stop();
        if (this->workerThread.joinable())
        {
            this->workerThread.join();
        }
        start();
        LOG_INFO(this->m_timerName + ": 定时器重置完成.");
    }
    bool isRunning()
    {
        return this->m_isRunning.load();
    }
    std::string getName() const
    {
        return this->m_timerName;
    }

    std::string m_timerName;
    Callback m_callback;
    TimeInterval m_interval;
    bool m_singleShot;
    std::atomic<bool> m_isRunning;
    std::thread workerThread;
    std::mutex m_mtx;
    std::condition_variable m_cv;
private:

    void workerLoop()
    {
        LOG_INFO(this->m_timerName + ": 工作线程启动. 线程ID: " + getThreadId());

        while (this->m_isRunning.load())
        {
            std::unique_lock<std::mutex> lock(this->m_mtx);

            bool isPredicateSatisfied = m_cv.wait_for(
                lock,
                this->m_interval,
                [this]() { return !this->m_isRunning.load(); }
            );

            if (isPredicateSatisfied || !this->m_isRunning.load())
            {
                break;
            }

            lock.unlock();
            try
            {
                if (this->m_callback)
                {
                    this->m_callback();
                }
            }
            catch (const std::exception& e)
            {
                LOG_ERROR(this->m_timerName + ": 回调执行异常: " + std::string(e.what()));
            }
            catch (...)
            {
                LOG_ERROR(this->m_timerName + ": 回调执行未知异常");
            }
            lock.lock();

            if (this->m_singleShot)
            {
                this->m_isRunning.store(false);
                break;
            }
        }

        this->m_isRunning.store(false);
        LOG_INFO(this->m_timerName + ": 工作线程退出. 线程ID: " + getThreadId());
    }

    std::string getThreadId() const
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return oss.str();
    }
};

Timer::Timer(const std::string& timerName, Callback callback, TimeInterval interval, bool singleShot)
{
    this->impl = std::make_unique<TimerImpl>(timerName, std::move(callback), interval, singleShot);
}

Timer::~Timer()
{
}

void Timer::start()
{
    this->impl->start();
}

void Timer::stop()
{
    this->impl->stop();
}

void Timer::reset()
{
    this->impl->reset();
}

bool Timer::isRunning()
{
    return this->impl->isRunning();
}

std::string Timer::getName() const
{
    return this->impl->getName();
}
