#pragma once
#include "./camera.h"
#include "./camera_utils.h"
#include "./stereo.h"
#include "./stereo_utils.h"
#include "./timer.h"
#include "./safe_queue.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
//-- XYZ3D相机相关
#include "LenaDDIUtility.h"
#include "stdafx.h"
//-- 热插拔相关
#include <windows.h>
#include <dbt.h>
#include <initguid.h>
#include <usbiodef.h>


struct WriteData
{
    std::string path; /*!< 路径 */
    cv::Mat frame;    /*!< 图像 */

    WriteData() = default;
    WriteData(const std::string& path, cv::Mat&& frame) :
        path(path),
        frame(std::move(frame))
    {
    }
    WriteData(std::string&& path, cv::Mat&& frame) :
        path(std::move(path)),
        frame(std::move(frame))
    {
    }
    WriteData(const WriteData&) = delete;
    WriteData& operator=(const WriteData&) = delete;
    WriteData(WriteData&& other) noexcept = default;
    WriteData& operator=(WriteData&& other) noexcept = default;
};

class camera::XYZ3D::XYZ3DImpl
{
public:
    XYZ3DImpl();
    ~XYZ3DImpl();

    void connectTask(const std::string& pidvid, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en);

    void captureTask();

    void liveTask();

    void writeTask();

    void videoTask(const std::string& path);

    void startWriteThread();

    void stopWriteThread();

    void hotplugTask();

    static LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

    void usbDeviceMonitorTask();

    void enableAE();

    void enableAWB();

    void disableAE();

    void disableAWB();

    float getExposureTime();

    float getISO();

    void setExposureTime(const float& t);

    void setISO(const float& iso);

    cv::Mat readFrame() const;

    static void onExpTimeChange(int pos, void* param);

    static void onISOChange(int pos, void* param);

    std::string getThreadID() const;

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
    std::atomic<ThreadTask> m_monitorTaskSignal; /*!< [monitor] 线程任务信号 */
    std::atomic<ThreadTask> m_capHotTaskSignal;  /*!< [hotplug] 热插拔线程任务信号 */
    std::atomic<ThreadTask> m_liveHotTaskSignal; /*!< [hotplug] 热插拔线程任务信号 */
    std::atomic<ThreadTask> m_captureTaskState;  /*!< [capture] 线程任务状态 */
    std::atomic<ThreadTask> m_liveTaskState;     /*!< [live] 线程任务状态 */
    std::atomic<ThreadTask> m_writeTaskState;    /*!< [write] 线程任务状态 */
    std::atomic<ThreadTask> m_videoTaskState;    /*!< [video] 线程任务状态 */
    std::atomic<ThreadTask> m_monitorTaskState;  /*!< [monitor] 线程任务状态 */
    std::atomic<bool> m_firstFrame;              /*!< [capture&live] 第1帧标志位 */
    std::atomic<bool> m_deviceOffline;           /*!< [monitor] USB设备离线标志位 */
    std::unique_ptr<cv::VideoCapture> m_cap;     /*!< 相机设备采集对象 */
    std::unique_ptr<cv::VideoWriter> m_video;    /*!< [video] 写入对象 */
    std::unique_ptr<Timer> m_scheduledTimer;     /*!< [scheduled] 定时器 */
    std::string m_liveWinName;                   /*!< [live] 窗口名称 */
    std::mutex m_captureMtx;                     /*!< [capture] 互斥锁 */
    std::mutex m_liveMtx;                        /*!< [live] 互斥锁 */
    std::mutex m_writeMtx;                       /*!< [write] 互斥锁 */
    std::mutex m_videoMtx;                       /*!< [video] 互斥锁 */
    std::mutex m_monitorMtx;                     /*!< [monitor] 互斥锁 */
    std::condition_variable m_captureCV;         /*!< [capture] 条件变量 */
    std::condition_variable m_liveCV;            /*!< [live] 条件变量 */
    std::condition_variable m_writeCV;           /*!< [write] 条件变量 */
    std::condition_variable m_videoCV;           /*!< [video] 条件变量 */
    std::condition_variable m_monitorCV;         /*!< [monitor] 条件变量 */
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
    std::string m_vidstr;                        /*!< [monitor] VID字符 */
    std::string m_pidstr;                        /*!< [monitor] PID字符 */
    std::string m_expBarName;                    /*!< [live] 窗口曝光时间滑动条名称 */
    std::string m_isoBarName;                    /*!< [live] 窗口ISO滑动条名称 */
    bool m_userBar;                              /*!< [live] 自定义滚动条 */
    HWND m_hwnd;                                 /*!< [monitor] Windows窗口句柄 */

    /// XYZ3D
    void* m_hLenaDDI;        /*!< [XYZ3D相机] 设备指针 */
    DEVSELINFO m_DevSelInfo; /*!< [XYZ3D相机] 设备选择信息 */

    /// Stereo Camera
    stereo::StereoParams m_stereoParams; /*!< 双目相机参数 */
    std::unique_ptr<stereo::EpipolarRectify> m_rectify; /*!< 极线校正对象 */
};
