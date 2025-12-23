/*********************************************************************
 * @file   XYZ3D_impl.h
 * @brief  茶叶嫩芽采摘双目相机采集图像模块实现类
 * @author Qianyao Zhuang
 * @date   May 2025
 *********************************************************************/
#pragma once
#include "../include/camera.h"
#include "../include/stereo.h"
#include <thread>
#include <atomic>
#include <shared_mutex>
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
//-- 条件唤醒
#include <condition_variable>
#include <mutex>

class camera::XYZ3D::XYZ3DImpl
{
public:
    XYZ3DImpl();
    ~XYZ3DImpl();
public:
    /**
     * @brief 连接相机任务.
     * @param [in] pid_vid_str 相机PID_VID字符串
     * @param [in] port        端口号, empty = no check
     * @param [in] imgsz       图像分辨率
     * @param [in] api         媒体API
     * @param [in] en          编码格式
     * @param [in] retry       连接相机重试次数
     */
    void connectTask(const std::string& pid_vid_str, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en);
    /**
     * @brief 实时采集任务.
     */
    void captureTask();
    /**
     * @brief 实时展示任务.
     */
    void liveTask(const bool& userBar);
    /**
     * @brief 热插拔任务.
     */
    void hotplugTask();
    /**
     * @brief 窗口消息处理回调函数.
     * @param [in] hwnd    窗口句柄
     * @param [in] message 消息
     * @param [in] wParam  附加信息
     * @param [in] lParam  附加信息
     * @return 状态
     */
    static LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);
    /**
     * @brief USB设备监视任务.
     */
    void usbDeviceMonitorTask();
    /**
     * @brief 开启AE.
     */
    void enableAE();
    /**
     * @brief 开启AWB.
     */
    void enableAWB();
    /**
     * @brief 关闭AE.
     */
    void disableAE();
    /**
     * @brief 关闭AWB.
     */
    void disableAWB();
    /**
     * @brief 获取曝光时间.
     * @return 曝光时间, ms
     */
    float getExposureTime();
    /**
     * @brief 获取曝光增益.
     * @return 曝光增益
     */
    float getISO();
    /**
     * @brief 设置曝光时间.
     * @param [in] t 曝光时间, ms
     */
    void setExposureTime(const float& t);
    /**
     * @brief 设置曝光增益.
     * @param [in] iso 曝光增益
     */
    void setISO(const float& iso);
    /**
     * @brief 读锁, 读取共享图像帧.
     * @return 共享图像帧
     */
    cv::Mat readFrame() const;
    /**
     * @brief 曝光时间滚动条事件回调函数.
     * @param [in] pos   滑动条位置
     * @param [in] param 用户参数
     */
    static void onExpTimeChange(int pos, void* param);
    /**
     * @brief 曝光增益滚动条事件回调函数.
     * @param [in] pos   滑动条位置
     * @param [in] param 用户参数
     */
    static void onISOChange(int pos, void* param);
    /**
     * @brief 获取相机连接状态.
     * @return 连接状态
     */
    bool isConnected() const;
public:
    ///// thread /////
    enum ThreadTask /*!< 线程任务状态枚举 */
    {
        STOP = 0, /*!< 停止 */
        RUN = 1, /*!< 运行 */
    };

    ThreadTask m_captureTaskSignal;     /*!< 实时采集图像线程任务信号 */
    ThreadTask m_liveTaskSignal;        /*!< 实时画面展示线程任务信号 */
    ThreadTask m_capHotplugTaskSignal;  /*!< 实时采集图像热插拔线程任务信号 */
    ThreadTask m_liveHotplugTaskSignal; /*!< 实时画面展示热插拔线程任务信号 */

    std::atomic<ThreadTask> m_captureTaskState; /*!< 实时采集线程任务状态 */
    std::atomic<ThreadTask> m_liveTaskState;    /*!< 实时展示线程任务状态 */
    std::atomic<bool> m_firstFrame;   /*!< 第1帧标志位 */
    std::atomic<int>  m_frameCount;   /*!< 图像帧计数 */

    std::string m_liveWinName;        /*!< 实时画面展示窗口名称 */
    std::string m_expBarName;         /*!< 实时画面展示窗口曝光时间滑动条名称 */
    std::string m_isoBarName;         /*!< 实时画面展示窗口ISO滑动条名称 */
    bool m_userBar;                   /*!< 是否使用用户自定义曝光时间与增益 */

    std::string m_vidstr;           /*!< VID字符 */
    std::string m_pidstr;           /*!< PID字符 */
    ThreadTask m_monitorTaskSignal; /*!< USB设备监视线程任务信号 */
    std::atomic<ThreadTask> m_monitorTaskState; /*!< USB设备监视线程任务状态 */
    std::atomic<bool> m_deviceOffline; /*!< USB设备离线标志位 */
    HWND m_hwnd;                       /*!< Windows窗口句柄 */


    StereoParams m_stereoParams;
    cv::VideoCapture* m_cap;          /*!< 相机设备采集对象 */
    MediaAPI m_api;
    VideoEncoding m_encoding;
    std::string m_vid_pid;
    std::string m_port;
    ImageSize m_imgsz;

    ///// XYZ3D /////
    void* m_hLenaDDI;                 /*!< 设备指针 */
    DEVSELINFO m_DevSelInfo;          /*!< 设备选择信息 */

    cv::Mat m_frameShared;            /*!< 线程共享图像帧 */
    std::unique_ptr<std::shared_mutex> m_sharedMutex; /*!< 线程共享数据锁 */

    std::mutex m_mtx;                 /*!< 互斥锁 */
    std::condition_variable m_cond;   /*!< 条件变量 -> 事件通知 */

    stereo::EpipolarRectify* m_rectify; /*!< 极线校正对象 */
};