/*********************************************************************
 * @file   camera.h
 * @brief  茶叶嫩芽采摘双目相机采集图像模块
 * @author Qianyao Zhuang
 * @date   May 2025
 * 
 * @details
 * 
 * @section 注意事项
 * 
 * @code demo 1 - xyz3d相机采集

#include <conio.h>
...
	camera::XYZ3D xyz3d;
	try
	{
		xyz3d.connect();
		xyz3d.startCaptureThread();
		xyz3d.startLiveThread(true);
		xyz3d.startUSBDEVMonitor();
	}
	catch (...)
	{
		return -1;
	}
	int key = -1;
	do
	{
		if (_kbhit())
		{
			char ch = _getch();
			switch (ch)
			{
			case 27://"ESC"
				xyz3d.release();
				key = 27;
				break;
			case 'c': case 'C':
			{
				break;
			}
			default:
				break;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	} while (key != 27);
 * @endcode demo 1 - xyz3d相机采集
 * 
 *********************************************************************/
#pragma once
#include "camera_utils.hpp"
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <memory>

namespace camera
{

class XYZ3D
{
public:
	XYZ3D();
	~XYZ3D();
public:
	/**
	 * @brief 加载双目参数文件.
	 * @param [in] stereoYAML 双目参数YAML文件, optional
	 */
	void loadStereoYAMLFile(const std::string& stereoYAML);
	/**
	 * @brief 连接相机.
	 * @param [in] pid_vid_str 相机PID_VID字符串
	 * @param [in] port        端口号
	 * @param [in] imgsz       图像分辨率
	 * @param [in] api         媒体API
	 * @param [in] en          编码格式
	 * @param [in] retry       连接相机重试次数
	 */
	void connect(const std::string& pid_vid_str = "vid_0211&pid_5838", const std::string& port = std::string(), camera::ImageSize imgsz = camera::ImageSize(2560, 720),
		const camera::MediaAPI& api = camera::MediaAPI::DSHOW, const camera::VideoEncoding& en = camera::VideoEncoding::YUY2, const int& retry = 3);
	/**
	 * @brief 释放当前相机.
	 */
	void release();
	/**
	 * @brief 启动实时采集线程.
	 * @throw 启动实时采集线程异常/超时
	 */
	void startCaptureThread();
	/**
	 * @brief 停止实时采集线程.
	 */
	void stopCaptureThread();
	/**
	 * @brief 启动实时展示线程.
	 * @param [in] userBar 是否展示曝光滚动条, 默认为false
	 * @throw 启动实时展示线程异常/超时
	 */
	void startLiveThread(const bool& userBar = false);
	/**
	 * @brief 停止实时展示线程.
	 */
	void stopLiveThread();
	/**
	 * @brief 启动USB设备插拔监视.
	 * @throw 启动USB设备插拔监视异常/超时
	 */
	void startUSBDEVMonitor();
	/**
	 * @brief 停止USB设备插拔监视.
	 */
	void stopUSBDEVMonitor();
	/**
	 * @brief 设置AE状态.
	 * @param [in] autoExposure 自动曝光状态, true为开启, false为关闭
	 */
	void setAE(const bool& autoExposure = true);
	/**
	 * @brief 设置AWB状态.
	 * @param [in] autoWhiteBalance 自动白平衡状态, true为开启, false为关闭
	 */
	void setAWB(const bool& autoWhiteBalance = true);
	/**
	 * @brief 获取当前相机曝光增益ISO.
	 * @return ISO
	 */
	float getISO();
	/**
	 * @brief 获取当前相机曝光时长EXPTIME.
	 * @return EXPTIME
	 */
	float getExposureTime();
	/**
	 * @brief 设置相机曝光增益ISO.
	 * @param [in] iso 曝光增益
	 */
	void setISO(const float& iso = 1.f);
	/**
	 * @brief 设置相机曝光时长.
	 * @param [in] exposureTime 曝光时长, ms. (!< Min = 0.037ms.)
	 */
	void setExposureTime(const float& exposureTime);
	/**
	 * @brief 捕获当前帧, 已开启实时采集线程可正常捕获.
	 * @return 当前帧双目图像, cv::Mat类型, 左右图像拼接, 未极线校正.
	 * @throw 捕获当前帧异常
	 */
	cv::Mat getRGBFrame();
	/**
	 * @brief 捕获当前帧, 已开启实时采集线程可正常捕获.
	 * @param [out] stereo 当前帧双目图像, cv::Mat类型, 左右图像拼接, 未极线校正.
	 * @param [out] left   极线校正后的左目图像
	 * @param [out] right  极线校正后的右目图像
	 * @param [in]  lrSwap 左右目图像交换(取决于相机是否反装), 默认为false
	 * @throw 捕获当前帧异常
	 */
	void getRGBFrame(cv::Mat& stereo, cv::Mat& left, cv::Mat& right, const bool& lrSwap = false);
	/**
	 * @brief 获取相机连接状态.
	 * @return 连接状态
	 */
	bool isConnected() const;

private:
	class XYZ3DImpl;
    std::unique_ptr<XYZ3DImpl> impl;
};

}
