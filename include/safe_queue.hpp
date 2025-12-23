#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <chrono>


template <typename T>
class SafeQueue
{
public:
	SafeQueue() :
		m_hasMaxSize(false), m_maxSize(0), m_running(true), m_queue() {
	};
	SafeQueue(size_t maxSize) :
		m_hasMaxSize(true), m_maxSize(maxSize), m_running(true), m_queue() {
		if (maxSize == 0) {
			m_hasMaxSize = false;
		}
	}
	SafeQueue(const SafeQueue& other) = delete;                 /*!< 禁止拷贝构造 */
	SafeQueue(SafeQueue&& other) noexcept = delete;;            /*!< 禁止移动构造 */
	SafeQueue& operator=(const SafeQueue& other) = delete;      /*!< 禁止拷贝赋值 */
	SafeQueue& operator=(SafeQueue&& other) noexcept = delete;; /*!< 禁止移动赋值 */
	~SafeQueue() {
		stop();
	}
	/**
	 * @brief 获取队首元素.
	 * @return 队首元素
	 */
	std::optional<T> front()
	{
		std::lock_guard<std::mutex> lock(this->m_mtx);
		if (this->m_queue.empty())
			return std::nullopt;
		return this->m_queue.front();
	}
	/**
	 * @brief 获取队首元素(const).
	 * @return 队首元素
	 */
	std::optional<T> front() const
	{
		std::lock_guard<std::mutex> lock(this->m_mtx);
		if (this->m_queue.empty())
			return std::nullopt;
		return this->m_queue.front();
	}
	/**
	 * @brief 获取队尾元素.
	 * @return 队尾元素
	 */
	std::optional<T> back()
	{
		std::lock_guard<std::mutex> lock(this->m_mtx);
		if (this->m_queue.empty())
			return std::nullopt;
		return this->m_queue.back();
	}
	/**
	 * @brief 获取队尾元素(const).
	 * @return 队尾元素
	 */
	std::optional<T> back() const
	{
		std::lock_guard<std::mutex> lock(this->m_mtx);
		if (this->m_queue.empty())
			return std::nullopt;
		return this->m_queue.back();
	}
	/**
	 * @brief 判断队列是否为空.
	 * @return 队列是否为空
	 */
	bool empty() const {
		std::lock_guard<std::mutex> lock(this->m_mtx);
		return this->m_queue.empty();
	}
	/**
	 * @brief 获取队列大小.
	 * @return 队列大小
	 */
	size_t size() const
	{
		std::lock_guard<std::mutex> lock(this->m_mtx);
		return this->m_queue.size();
	}
	/**
	 * @brief 入队(右值引用).
	 * @param [in] item 入队元素
	 */
	void push(T&& item)
	{
		std::unique_lock<std::mutex> lock(this->m_mtx);

		if (this->m_hasMaxSize)
		{
			this->m_notFullCV.wait(lock, [this]() {
				return this->m_queue.size() < this->m_maxSize || !this->m_running;
			});
		}

		if (!this->m_running)
			return;

		this->m_queue.push(std::move(item));
		this->m_notEmptyCV.notify_one();
	}
	/**
	 * @brief 入队(左值引用).
	 * @param [in] item 入队元素
	 */
	void push(const T& item)
	{
		std::unique_lock<std::mutex> lock(this->m_mtx);

		if (this->m_hasMaxSize)
		{
			this->m_notFullCV.wait(lock, [this]() {
				return this->m_queue.size() < this->m_maxSize || !this->m_running;
			});
		}

		if (!this->m_running)
			return;

		this->m_queue.push(item);
		this->m_notEmptyCV.notify_one();
	}
	/**
	 * @brief 批量入队(右值引用).
	 * @param [in] items 批量入队元素
	 */
	void pushBatch(std::vector<T>&& items)
	{
		if (items.empty())
			return;

		std::unique_lock<std::mutex> lock(this->m_mtx);

		if (!this->m_running)
			return;

		if (this->m_hasMaxSize)
		{
			size_t remaining = items.size();
			auto it = items.begin();
			while (remaining > 0 && this->m_running)
			{
				// 等待队列有剩余容量
				this->m_notFullCV.wait(lock, [this]() {
					return this->m_queue.size() < this->m_maxSize || !this->m_running;
					});

				if (!this->m_running)
					return;

				// 计算可批量插入的元素数量
				size_t available = this->m_maxSize - this->m_queue.size();
				size_t batchSize = std::min(available, remaining);

				// 批量插入
				for (size_t i = 0; i < batchSize; ++i, ++it, --remaining)
				{
					this->m_queue.push(std::move(*it));
				}
			}
		}
		else
		{
			for (auto&& item : items)
			{
				if (!this->m_running)
					return;
				this->m_queue.push(std::move(item));
			}
		}

		this->m_notEmptyCV.notify_all();
	}
	/**
	 * @brief 批量入队(左值引用).
	 * @param [in] items 批量入队元素
	 */
	void pushBatch(const std::vector<T>& items)
	{
		if (items.empty())
			return;

		std::unique_lock<std::mutex> lock(this->m_mtx);

		if (!this->m_running)
			return;

		if (this->m_hasMaxSize)
		{
			size_t remaining = items.size();
			auto it = items.cbegin(); // 左值使用const迭代器
			while (remaining > 0 && this->m_running)
			{
				this->m_notFullCV.wait(lock, [this]() {
					return this->m_queue.size() < this->m_maxSize || !this->m_running;
					});

				if (!this->m_running)
					return;

				// 计算可批量插入的数量
				size_t available = this->m_maxSize - this->m_queue.size();
				size_t batchSize = std::min(available, remaining);

				// 批量插入左值元素
				for (size_t i = 0; i < batchSize; ++i, ++it, --remaining)
				{
					this->m_queue.push(*it);
				}
			}
		}
		else
		{
			for (const auto& item : items)
			{
				if (!this->m_running)
					return;
				this->m_queue.push(item);
			}
		}

		this->m_notEmptyCV.notify_all();
	}
	/**
	 * @brief 入队(右值引用). 队列满时丢弃旧帧, 推入新帧(非阻塞)
	 * @param [in] item 入队元素
	 */
	void pushWithDropOld(T&& item)
	{
		std::unique_lock<std::mutex> lock(this->m_mtx);
		if (!this->m_running)
			return;

		// 队列满时, 丢弃队首
		if (this->m_hasMaxSize && this->m_queue.size() >= this->m_maxSize) 
		{
			this->m_queue.pop();
		}

		this->m_queue.push(std::move(item));
		this->m_notEmptyCV.notify_one();
	}
	/**
	 * @brief 入队(左值引用). 队列满时丢弃旧帧, 推入新帧(非阻塞)
	 * @param [in] item 入队元素
	 */
	void pushWithDropOld(const T& item)
	{
		std::unique_lock<std::mutex> lock(this->m_mtx);
		if (!this->m_running)
			return;

		// 队列满时, 丢弃队首
		if (this->m_hasMaxSize && this->m_queue.size() >= this->m_maxSize) 
		{
			this->m_queue.pop();
		}

		this->m_queue.push(item);
		this->m_notEmptyCV.notify_one();
	}
	/**
	 * @brief 出队(阻塞等待队列非空或线程停止)
	 */
	void pop()
	{
		std::unique_lock<std::mutex> lock(this->m_mtx);

		this->m_notEmptyCV.wait(lock, [this]() {
			return !this->m_queue.empty() || !this->m_running;
		});

		if (!this->m_running && this->m_queue.empty())
			return;

		this->m_queue.pop();

		if (this->m_hasMaxSize)
			this->m_notFullCV.notify_one();
	}
	/**
	 * @brief 返回队首元素并出队(阻塞等待队列非空或线程停止)
	 * @return 队首元素(队列停止且为空时返回std::nullopt)
	 */
	std::optional<T> frontAndPop()
	{
		std::unique_lock<std::mutex> lock(this->m_mtx);

		this->m_notEmptyCV.wait(lock, [this]() {
			return !this->m_queue.empty() || !this->m_running;
		});

		if (!this->m_running && this->m_queue.empty())
			return std::nullopt;

		T frontItem = std::move(this->m_queue.front());
		this->m_queue.pop();

		if (this->m_hasMaxSize)
			this->m_notFullCV.notify_one();

		return frontItem;
	}
	/**
	 * @brief 返回队首元素并出队(阻塞等待队列非空或线程停止, 超时返回std::nullopt)
	 * @param [in] timeout 等待超时时间
	 * @return	队首元素(队列停止且为空时 或 超时返回std::nullopt)
	 */
	std::optional<T> frontAndPop(std::chrono::milliseconds timeout)
	{
		std::unique_lock<std::mutex> lock(this->m_mtx);
		if (!this->m_notEmptyCV.wait_for(lock, timeout, [this]() {
			return !this->m_queue.empty() || !this->m_running;
		})) 
		{
			return std::nullopt; // 超时返回空
		}

		if (!this->m_running && this->m_queue.empty())
			return std::nullopt;

		T frontItem = std::move(this->m_queue.front());
		this->m_queue.pop();

		if (this->m_hasMaxSize)
			this->m_notFullCV.notify_one();

		return frontItem;
	}
	/**
	 * @brief 非阻塞出队(队列空时直接返回false)
	 * @return 出队成功返回true, 队列空或已停止返回false
	 */
	bool tryPop()
	{
		std::lock_guard<std::mutex> lock(this->m_mtx);

		if (this->m_queue.empty() || !this->m_running)
			return false;

		this->m_queue.pop();

		if (this->m_hasMaxSize)
			this->m_notFullCV.notify_one();

		return true;
	}
	/**
	 * @brief 非阻塞返回队首元素并出队
	 * @return 队首元素(队列空或已停止返回std::nullopt)
	 */
	std::optional<T> tryFrontAndPop()
	{
		std::lock_guard<std::mutex> lock(this->m_mtx);

		if (this->m_queue.empty() || !this->m_running)
			return std::nullopt;

		T frontItem = std::move(this->m_queue.front());
		this->m_queue.pop();

		if (this->m_hasMaxSize)
			this->m_notFullCV.notify_one();

		return frontItem;
	}
	/**
	 * @brief 停止队列(唤醒所有等待的变量)
	 */
	void stop()
	{
		std::lock_guard<std::mutex> lock(this->m_mtx);
		this->m_running = false;
		this->m_notEmptyCV.notify_all();
		this->m_notFullCV.notify_all();
	}
	/**
	 * @brief 清空队列所有元素
	 * @note 仅清空元素, 不改变队列的运行状态
	 */
	void clear()
	{
		std::lock_guard<std::mutex> lock(this->m_mtx);

		std::queue<T>().swap(m_queue);

		if (this->m_hasMaxSize)
		{
			this->m_notFullCV.notify_all();
		}
	}
	/**
	 * @brief 查询队列是否有最大长度限制.
	 * @return 队列是否有最大长度限制
	 */
	bool hasMaxSize() const { return this->m_hasMaxSize; }
	/**
	 * @brief 获取队列最大长度.
	 * @return 队列最大长度
	 */
	size_t getMaxSize() const { return this->m_maxSize; }

private:
	std::queue<T> m_queue;                /*!< 数据队列 */
	mutable std::mutex m_mtx;             /*!< 互斥锁 */
	std::condition_variable m_notEmptyCV; /*!< 队列非空条件变量 */
	std::condition_variable m_notFullCV;  /*!< 队列未满条件变量 */
	bool m_hasMaxSize;                    /*!< 队列是否有限长度 */
	size_t m_maxSize;                     /*!< 队列最大长度 */
	bool m_running;                       /*!< 队列运行状态 */
};
