#pragma once
#include "./camera.h"
#include "./camera_utils.h"
#include "./timer.h"
#include "./safe_queue.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

struct WriteData
{
    std::string path; /*!< 路径 */
    cv::Mat frame;    /*!< 图像 */

    WriteData() = default;
    WriteData(const std::string& path, cv::Mat&& frame) :
        path(path),
        frame(std::move(frame))
    {}
    WriteData(std::string&& path, cv::Mat&& frame) :
        path(std::move(path)),
        frame(std::move(frame))
    {}
    WriteData(const WriteData&) = delete;
    WriteData& operator=(const WriteData&) = delete;
    WriteData(WriteData&& other) noexcept = default;
    WriteData& operator=(WriteData&& other) noexcept = default;
};

class camera::WebCamera::WebCameraImpl
{
public:
    WebCameraImpl();
    ~WebCameraImpl();
    /**
     * @brief 连接相机任务.
     * @param [in] pidvid 相机PID_VID字符串
     * @param [in] port   端口号, empty = no check
     * @param [in] imgsz  图像分辨率
     * @param [in] api    媒体API
     * @param [in] en     编码格式
     * @param [in] retry  连接相机重试次数
     */
    void connectTask(const std::string& pidvid, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en);
    /**
     * @brief 实时采集任务.
     */
    void captureTask();
    /**
     * @brief 实时展示任务.
     */
    void liveTask();
    /**
     * @brief 保存图像任务.
     */
    void writeTask();
    /**
     * @brief 录像任务.
     */
    void videoTask(const std::string& path);
    /**
     * @brief 启动保存图像线程.
     */
    void startWriteThread();
    /**
     * @brief 停止保存图像线程.
     */
    void stopWriteThread();
    /**
     * @brief 读取图像帧.
     * @return 图像帧
     */
    cv::Mat readFrame();
    /**
     * @brief 获取线程ID.
     * @return 线程ID字符串
     */
    std::string getThreadID() const;
    /**
     * @brief 获取相机连接状态.
     * @return 连接状态
     */
    bool isConnected() const;

    enum ThreadTask /*!< 线程任务状态 */
    {
        STOP = 0, /*!< 停止 */
        RUN  = 1, /*!< 运行 */
    };
    std::thread m_captureThread;                 /*!< [capture] 线程, producer */
    std::thread m_liveThread;                    /*!< [live] 线程, consumer */
    std::thread m_writeThread;                   /*!< [write] 线程 */
    std::thread m_videoThread;                   /*!< [video] 线程 */
    std::atomic<ThreadTask> m_captureTaskSignal; /*!< [capture] 线程任务信号 */
    std::atomic<ThreadTask> m_liveTaskSignal;    /*!< [live] 线程任务信号 */
    std::atomic<ThreadTask> m_writeTaskSignal;   /*!< [write] 线程任务信号 */
    std::atomic<ThreadTask> m_videoTaskSignal;   /*!< [video] 线程任务信号 */
    std::atomic<ThreadTask> m_captureTaskState;  /*!< [capture] 线程任务状态 */
    std::atomic<ThreadTask> m_liveTaskState;     /*!< [live] 线程任务状态 */
    std::atomic<ThreadTask> m_writeTaskState;    /*!< [write] 线程任务状态 */
    std::atomic<ThreadTask> m_videoTaskState;    /*!< [video] 线程任务状态 */
    std::atomic<bool> m_firstFrame;              /*!< [capture&live] 第1帧标志位 */
    std::unique_ptr<cv::VideoCapture> m_cap;     /*!< 相机设备采集对象 */
    std::unique_ptr<cv::VideoWriter> m_video;    /*!< [video] 写入对象 */
    std::unique_ptr<Timer> m_scheduledTimer;     /*!< [scheduled] 定时器 */
    std::string m_liveWinName;                   /*!< [live] 窗口名称 */
    std::mutex m_captureMtx;                     /*!< [capture] 互斥锁 */
    std::mutex m_liveMtx;                        /*!< [live] 互斥锁 */
    std::mutex m_writeMtx;                       /*!< [write] 互斥锁 */
    std::mutex m_videoMtx;                       /*!< [video] 互斥锁 */
    std::condition_variable m_captureCV;         /*!< [capture] 条件变量 */
    std::condition_variable m_liveCV;            /*!< [live] 条件变量 */
    std::condition_variable m_writeCV;           /*!< [write] 条件变量 */
    std::condition_variable m_videoCV;           /*!< [video] 条件变量 */
    SafeQueue<cv::Mat> m_frameQueue;             /*!< [capture&live] 缓冲队列, producer queue */
    SafeQueue<WriteData> m_writeQueue;           /*!< [write] 保存队列, 保存和定时保存共用 */
    SafeQueue<cv::Mat> m_videoQueue;             /*!< [video] 视频队列 */
    cv::Mat m_curFrame;                          /*!< [capture] 当前帧 */
    std::unique_ptr<std::shared_mutex> m_curMtx; /*!< [capture] 当前帧互斥锁 */
    MediaAPI m_api;                              /*!< [connect] 相机调用API */
    VideoEncoding m_encoding;                    /*!< [connect] 相机编码格式 */
    std::string m_vidpid;                        /*!< [connect] 相机VIDPID */
    std::string m_port;                          /*!< [connect] 相机端口 */
    ImageSize m_imgsz;                           /*!< [connect] 图像尺寸 */
   
};
