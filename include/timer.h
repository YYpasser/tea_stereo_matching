#pragma once
#include <string>
#include <memory>
#include <functional>
#include <chrono>

/**
 * @brief 定时器.
 * @example
 * 
	using namespace std::literals;
	Timer loopTimer(
		"循环定时器",
		[]() {
			printf("Hello Timer\n");
		},
		1s,
		false
	);
		loopTimer.start();

	for (int i = 1; i <= 5; ++i)
	{
		std::this_thread::sleep_for(1s);
		printf("[主线程] 执行第%d秒任务\n", i);
	}

	loopTimer.stop();
 *
 */
class Timer
{
public:
	using TimeInterval = std::chrono::milliseconds;
	using Callback = std::function<void()>;
	/**
	 * @brief 定时器构造函数.
	 * @param [in] timerName  定时器名称
	 * @param [in] callback   回调函数
	 * @param [in] interval   间隔时间
	 * @param [in] singleShot 是否单次定时
	 */
	Timer(const std::string& timerName, Callback callback, TimeInterval interval, bool singleShot = false);
	Timer(const Timer& other) = delete;
    Timer(Timer&& other) = default;
    Timer& operator=(const Timer& other) = delete;
    Timer& operator=(Timer&& other) = default;
	~Timer();
	/**
	 * @brief 启动定时器
	 */
    void start();
	/**
	 * @brief 停止定时器
	 */
	void stop();
	/**
	 * @brief 重置定时器
	 */
	void reset();
	/**
	 * @brief 定时器是否正在运行.
	 * @return 定时器运行状态.
	 */
	bool isRunning();
	/**
	 * @brief 获取定时器名称.
	 * @return 定时器名称.
	 */
	std::string getName() const;

private:
	class TimerImpl;
	std::unique_ptr<TimerImpl> impl;
};
